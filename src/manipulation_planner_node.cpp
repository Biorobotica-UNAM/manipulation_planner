#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit_msgs/srv/get_motion_plan.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include "manipulation_planner_interfaces/srv/plan_trajectory.hpp"
#include <algorithm>
#include <future>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <thread>
#include <unordered_map>
#include <unordered_set>

using std::placeholders::_1;
using std::placeholders::_2;
using PlanTrajectory = manipulation_planner_interfaces::srv::PlanTrajectory;
using GetMotionPlan  = moveit_msgs::srv::GetMotionPlan;

class ManipulationPlannerNode : public rclcpp::Node
{
public:
    ManipulationPlannerNode() : Node("manipulation_planner_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing manipulation_planner_node...");

        // El filtro de nube vive en manipulation_tools y normalmente esta
        // apagado. Este planner lo activa solo durante la ventana de planeacion
        // para que MoveIt actualice su octomap con el ROI de manipulacion.
        this->declare_parameter<bool>("use_cloud_filter", true);
        this->declare_parameter<bool>("clear_octomap_on_plan", true);
        this->declare_parameter<std::string>(
            "cloud_filter_enable_service", "/manipulation/cloud_filter/enable");
        this->declare_parameter<std::string>(
            "cloud_filter_node", "/manipulation/cloud_filter");
        this->declare_parameter<std::string>(
            "clear_octomap_service", "/manipulation/clear_octomap");
        this->declare_parameter<double>("cloud_filter_warmup_sec", 1.2);
        this->declare_parameter<double>("cloud_filter_service_timeout_sec", 1.5);
        this->declare_parameter<double>("goal_position_tolerance_m", 0.015);
        this->declare_parameter<double>("goal_orientation_tolerance_rad", 0.10);

        use_cloud_filter_ = this->get_parameter("use_cloud_filter").as_bool();
        clear_octomap_on_plan_ = this->get_parameter("clear_octomap_on_plan").as_bool();
        cloud_filter_enable_service_ =
            this->get_parameter("cloud_filter_enable_service").as_string();
        cloud_filter_node_ =
            this->get_parameter("cloud_filter_node").as_string();
        clear_octomap_service_ =
            this->get_parameter("clear_octomap_service").as_string();
        cloud_filter_warmup_sec_ =
            this->get_parameter("cloud_filter_warmup_sec").as_double();
        cloud_filter_service_timeout_sec_ =
            this->get_parameter("cloud_filter_service_timeout_sec").as_double();
        goal_position_tolerance_m_ =
            this->get_parameter("goal_position_tolerance_m").as_double();
        goal_orientation_tolerance_rad_ =
            this->get_parameter("goal_orientation_tolerance_rad").as_double();

        client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        srv_cb_group_    = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/manipulation/joint_states", 10,
            std::bind(&ManipulationPlannerNode::jointStateCallback, this, _1));

        plan_client_ = this->create_client<GetMotionPlan>(
            "/manipulation/plan_kinematic_path",
            rmw_qos_profile_services_default, client_cb_group_);

        cloud_filter_client_ = this->create_client<std_srvs::srv::SetBool>(
            cloud_filter_enable_service_,
            rmw_qos_profile_services_default,
            client_cb_group_);

        clear_octomap_client_ = this->create_client<std_srvs::srv::Empty>(
            clear_octomap_service_,
            rmw_qos_profile_services_default,
            client_cb_group_);
        cloud_filter_params_client_ =
            std::make_shared<rclcpp::AsyncParametersClient>(this, cloud_filter_node_);

        srv_ = this->create_service<PlanTrajectory>(
            "plan_trajectory",
            std::bind(&ManipulationPlannerNode::planTrajectory, this, _1, _2),
            rmw_qos_profile_services_default, srv_cb_group_);

        RCLCPP_INFO(this->get_logger(), "Manipulation planner ready");
    }

private:
    rclcpp::Service<PlanTrajectory>::SharedPtr                    srv_;
    rclcpp::Client<GetMotionPlan>::SharedPtr                      plan_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr             cloud_filter_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr               clear_octomap_client_;
    std::shared_ptr<rclcpp::AsyncParametersClient>                cloud_filter_params_client_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::CallbackGroup::SharedPtr                              client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr                              srv_cb_group_;

    sensor_msgs::msg::JointState last_joint_state_;
    bool                         joint_state_received_ = false;
    std::mutex                   joint_state_mutex_;

    bool        use_cloud_filter_ = true;
    bool        clear_octomap_on_plan_ = true;
    std::string cloud_filter_enable_service_;
    std::string cloud_filter_node_;
    std::string clear_octomap_service_;
    double      cloud_filter_warmup_sec_ = 1.2;
    double      cloud_filter_service_timeout_sec_ = 1.5;
    double      goal_position_tolerance_m_ = 0.015;
    double      goal_orientation_tolerance_rad_ = 0.10;

    struct PlanState
    {
        std::mutex                                          mtx;
        std::condition_variable                             cv;
        bool                                                done = false;
        moveit_msgs::srv::GetMotionPlan::Response::SharedPtr result;
    };

    // ─────────────────────────────────────────────────────────────
    //  tipLink
    // ─────────────────────────────────────────────────────────────
    std::string tipLink(const std::string& group,
                        const std::string& requested_tip) const
    {
        static const std::unordered_map<
            std::string, std::unordered_set<std::string>> valid_tips = {
            {"left_arm_group",  {"left_grab_object_link",  "left_pinch_object_link"}},
            {"right_arm_group", {"right_grab_object_link", "right_pinch_object_link"}},
        };

        auto group_it = valid_tips.find(group);
        if (group_it == valid_tips.end())
        {
            RCLCPP_ERROR(this->get_logger(),
                "Unknown planning group '%s'. Valid groups: left_arm_group, right_arm_group.",
                group.c_str());
            return "";
        }

        const auto& allowed = group_it->second;
        if (requested_tip.empty() || allowed.find(requested_tip) == allowed.end())
        {
            RCLCPP_ERROR(this->get_logger(),
                "Invalid or missing tip_link '%s' for group '%s'. "
                "Valid tips for this group: %s.",
                requested_tip.c_str(), group.c_str(),
                [&]() {
                    std::string s;
                    for (const auto& t : allowed) s += t + " ";
                    return s;
                }().c_str());
            return "";
        }

        return requested_tip;
    }

    // ─────────────────────────────────────────────────────────────
    //  jointStateCallback
    // ─────────────────────────────────────────────────────────────
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        last_joint_state_     = *msg;
        joint_state_received_ = true;
    }

    bool setCloudFilter(bool enabled)
    {
        // Servicio /manipulation/cloud_filter/enable: al habilitarlo, el filtro
        // se suscribe al Kinect; al deshabilitarlo, suelta la suscripcion.
        if (!use_cloud_filter_)
            return true;

        const auto timeout =
            std::chrono::duration<double>(std::max(0.0, cloud_filter_service_timeout_sec_));
        if (!cloud_filter_client_->wait_for_service(
                std::chrono::duration_cast<std::chrono::nanoseconds>(timeout)))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Manipulation cloud filter service unavailable: %s",
                cloud_filter_enable_service_.c_str());
            return false;
        }

        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = enabled;
        RCLCPP_INFO(
            this->get_logger(),
            "Requesting manipulation cloud filter %s",
            enabled ? "enable" : "disable");

        auto future = cloud_filter_client_->async_send_request(req);
        if (future.wait_for(
                std::chrono::duration_cast<std::chrono::nanoseconds>(timeout)) !=
            std::future_status::ready)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Timeout while setting manipulation cloud filter to %s",
                enabled ? "enabled" : "disabled");
            return false;
        }

        const auto response = future.get();
        if (!response->success)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Manipulation cloud filter rejected request: %s",
                response->message.c_str());
            return false;
        }
        RCLCPP_INFO(
            this->get_logger(),
            "Manipulation cloud filter %s confirmed: %s",
            enabled ? "enable" : "disable",
            response->message.c_str());
        return true;
    }

    void clearOctomap()
    {
        // Antes de cada plan limpiamos el octomap para no arrastrar obstaculos
        // viejos de una pose/escena anterior.
        if (!clear_octomap_on_plan_)
            return;

        const auto timeout =
            std::chrono::duration<double>(std::max(0.0, cloud_filter_service_timeout_sec_));
        if (!clear_octomap_client_->wait_for_service(
                std::chrono::duration_cast<std::chrono::nanoseconds>(timeout)))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "MoveIt clear_octomap service unavailable: %s",
                clear_octomap_service_.c_str());
            return;
        }

        auto req = std::make_shared<std_srvs::srv::Empty::Request>();
        RCLCPP_INFO(this->get_logger(), "Clearing MoveIt octomap before planning");
        auto future = clear_octomap_client_->async_send_request(req);
        if (future.wait_for(
                std::chrono::duration_cast<std::chrono::nanoseconds>(timeout)) !=
            std::future_status::ready)
        {
            RCLCPP_WARN(this->get_logger(), "Timeout while clearing MoveIt octomap");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "MoveIt octomap clear request completed");
    }

    void configureTargetExclusion(
        bool enabled,
        const geometry_msgs::msg::PoseStamped& center,
        double radius_xy,
        double height_z)
    {
        // La exclusion se manda como parametros porque el filtro es quien ve la
        // nube. El centro viene de Take en base_link, usando la pose original
        // del objeto antes del offset de agarre.
        if (!use_cloud_filter_)
            return;

        if (!cloud_filter_params_client_->wait_for_service(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(
                        std::max(0.0, cloud_filter_service_timeout_sec_)))))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Manipulation cloud filter parameter service unavailable: %s",
                cloud_filter_node_.c_str());
            return;
        }

        std::vector<rclcpp::Parameter> params;
        params.emplace_back("target_exclusion_enabled", enabled);
        params.emplace_back("target_exclusion_x", center.pose.position.x);
        params.emplace_back("target_exclusion_y", center.pose.position.y);
        params.emplace_back("target_exclusion_z", center.pose.position.z);
        params.emplace_back("target_exclusion_radius_xy", std::max(0.0, radius_xy));
        params.emplace_back("target_exclusion_height_z", std::max(0.0, height_z));

        auto future = cloud_filter_params_client_->set_parameters(params);
        const auto timeout =
            std::chrono::duration<double>(std::max(0.0, cloud_filter_service_timeout_sec_));
        if (future.wait_for(
                std::chrono::duration_cast<std::chrono::nanoseconds>(timeout)) !=
            std::future_status::ready)
        {
            RCLCPP_WARN(this->get_logger(), "Timeout while configuring target exclusion");
            return;
        }

        const auto results = future.get();
        bool accepted = true;
        for (const auto& result : results)
            accepted = accepted && result.successful;

        if (!accepted)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "At least one target exclusion parameter was rejected by %s",
                cloud_filter_node_.c_str());
        }

        if (enabled)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Target exclusion enabled center=[%.3f %.3f %.3f] radius_xy=%.3f height_z=%.3f",
                center.pose.position.x,
                center.pose.position.y,
                center.pose.position.z,
                std::max(0.0, radius_xy),
                std::max(0.0, height_z));
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Target exclusion disabled");
        }
    }

    void prepareCloudFilterForPlanning(const PlanTrajectory::Request& request)
    {
        if (!use_cloud_filter_)
            return;

        // Orden intencional: prender filtro, limpiar octomap, esperar unas
        // muestras filtradas y entonces llamar a MoveIt.
        RCLCPP_INFO(
            this->get_logger(),
            "Preparing cloud filter for planning: exclusion=%s warmup=%.2f s timeout=%.2f s",
            request.enable_target_exclusion ? "enabled" : "disabled",
            cloud_filter_warmup_sec_,
            cloud_filter_service_timeout_sec_);
        configureTargetExclusion(
            request.enable_target_exclusion,
            request.target_exclusion_center,
            request.target_exclusion_radius_xy,
            request.target_exclusion_height_z);
        setCloudFilter(true);
        clearOctomap();

        const auto warmup =
            std::chrono::duration<double>(std::max(0.0, cloud_filter_warmup_sec_));
        std::this_thread::sleep_for(
            std::chrono::duration_cast<std::chrono::nanoseconds>(warmup));
        RCLCPP_INFO(this->get_logger(), "Cloud filter warmup completed");
    }

    void finishCloudFilterForPlanning()
    {
        // Al terminar la respuesta de MoveIt, el Kinect deja de pasar por este
        // filtro dedicado para no gastar recursos fuera de manipulacion.
        if (!use_cloud_filter_)
            return;
        RCLCPP_INFO(this->get_logger(), "Finishing cloud filter planning window");
        setCloudFilter(false);
        geometry_msgs::msg::PoseStamped empty_center;
        configureTargetExclusion(false, empty_center, 0.0, 0.0);
    }

    // ─────────────────────────────────────────────────────────────
    //  planTrajectory
    // ─────────────────────────────────────────────────────────────
    void planTrajectory(
        const std::shared_ptr<PlanTrajectory::Request>  request,
        std::shared_ptr<PlanTrajectory::Response>       response)
    {
        const std::string& group = request->planning_group.empty()
            ? "right_arm_group" : request->planning_group;

        RCLCPP_INFO(this->get_logger(),
            "Planning request received — group: %s | tip: %s",
            group.c_str(), request->tip_link.c_str());

        // ── Validate tip ──────────────────────────────────────────
        const std::string tip = tipLink(group, request->tip_link);
        if (tip.empty())
        {
            response->success = false;
            response->message = "Invalid tip_link '" + request->tip_link +
                                "' for group '" + group + "'";
            return;
        }

        // ── Check joint states ────────────────────────────────────
        sensor_msgs::msg::JointState current_joint_state;
        {
            std::lock_guard<std::mutex> lock(joint_state_mutex_);
            if (!joint_state_received_)
            {
                response->success = false;
                response->message = "No joint_states received yet";
                return;
            }
            current_joint_state = last_joint_state_;
        }

        // ── Check MoveIt service ──────────────────────────────────
        if (!plan_client_->wait_for_service(std::chrono::seconds(2)))
        {
            response->success = false;
            response->message = "MoveIt planning service unavailable";
            return;
        }

        prepareCloudFilterForPlanning(*request);

        // ── Build request ─────────────────────────────────────────
        auto srv_req = std::make_shared<GetMotionPlan::Request>();
        auto& req    = srv_req->motion_plan_request;

        req.group_name            = group;
        req.allowed_planning_time = request->planning_time > 0.0
                                        ? request->planning_time : 5.0;
        req.num_planning_attempts = 10;
        req.start_state.joint_state = current_joint_state;

        req.workspace_parameters.header.frame_id = "base_link";
        req.workspace_parameters.min_corner.x = -2.0;
        req.workspace_parameters.min_corner.y = -2.0;
        req.workspace_parameters.min_corner.z = -2.0;
        req.workspace_parameters.max_corner.x =  2.0;
        req.workspace_parameters.max_corner.y =  2.0;
        req.workspace_parameters.max_corner.z =  2.0;

        const double position_tolerance =
            std::max(0.001, goal_position_tolerance_m_);
        const double orientation_tolerance =
            std::max(0.001, goal_orientation_tolerance_rad_);

        RCLCPP_INFO(
            this->get_logger(),
            "MoveIt goal target=[%.3f %.3f %.3f] orientation=[%.3f %.3f %.3f %.3f] position_tol=%.3f orientation_tol=%.3f",
            request->target_pose.pose.position.x,
            request->target_pose.pose.position.y,
            request->target_pose.pose.position.z,
            request->target_pose.pose.orientation.x,
            request->target_pose.pose.orientation.y,
            request->target_pose.pose.orientation.z,
            request->target_pose.pose.orientation.w,
            position_tolerance,
            orientation_tolerance);

        // Position constraint
        moveit_msgs::msg::PositionConstraint pos;
        pos.header    = request->target_pose.header;
        pos.link_name = tip;
        pos.weight    = 1.0;

        shape_msgs::msg::SolidPrimitive sphere;
        sphere.type       = shape_msgs::msg::SolidPrimitive::SPHERE;
        // Radio de aceptacion alrededor de la pose objetivo. Vision puede tener
        // pequenos errores; una esfera demasiado pequena impide que MoveIt
        // samplee estados validos aunque el agarre sea fisicamente aceptable.
        sphere.dimensions = {position_tolerance};
        pos.constraint_region.primitives.push_back(sphere);
        pos.constraint_region.primitive_poses.push_back(request->target_pose.pose);

        // Orientation constraint
        moveit_msgs::msg::OrientationConstraint ori;
        ori.header                    = request->target_pose.header;
        ori.link_name                 = tip;
        ori.orientation               = request->target_pose.pose.orientation;
        // Tolerancia angular en radianes. La orientacion de vision describe la
        // intencion de agarre, pero no debe ser tan estricta que bloquee IK.
        ori.absolute_x_axis_tolerance = orientation_tolerance;
        ori.absolute_y_axis_tolerance = orientation_tolerance;
        ori.absolute_z_axis_tolerance = orientation_tolerance;
        ori.weight                    = 1.0;

        moveit_msgs::msg::Constraints goal;
        goal.position_constraints.push_back(pos);
        goal.orientation_constraints.push_back(ori);
        req.goal_constraints.push_back(goal);

        auto state = std::make_shared<PlanState>();

        plan_client_->async_send_request(srv_req,
            [state](rclcpp::Client<GetMotionPlan>::SharedFuture future)
            {
                std::lock_guard<std::mutex> lock(state->mtx);
                state->result = future.get();
                state->done   = true;
                state->cv.notify_one();
            });

        const int timeout_s = static_cast<int>(req.allowed_planning_time) + 1;

        std::unique_lock<std::mutex> lock(state->mtx);
        if (!state->cv.wait_for(lock,
                std::chrono::seconds(timeout_s),
                [&state]{ return state->done; }))
        {
            finishCloudFilterForPlanning();
            response->success = false;
            response->message = "Planning timeout";
            RCLCPP_ERROR(this->get_logger(), "Planning timeout after %d s", timeout_s);
            return;
        }

        finishCloudFilterForPlanning();

        // ── Evaluate response ─────────────────────────────────────
        if (!state->result ||
            state->result->motion_plan_response.error_code.val !=
            moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            response->success = false;
            response->message = state->result
                ? "Planning failed with error code: " +
                  std::to_string(state->result->motion_plan_response.error_code.val)
                : "Null response from planner";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        const auto& traj =
            state->result->motion_plan_response.trajectory.joint_trajectory;

        if (traj.points.empty())
        {
            response->success = false;
            response->message = "Empty trajectory";
            return;
        }

        RCLCPP_INFO(this->get_logger(),
            "Plan found: %zu waypoints, %zu joints",
            traj.points.size(), traj.joint_names.size());

        response->joint_names = traj.joint_names;
        for (const auto& p : traj.points)
            for (double p_val : p.positions)
                response->waypoints.push_back(p_val);

        RCLCPP_INFO(this->get_logger(),
            "Flattened trajectory: %zu points × %zu joints = %zu values",
            traj.points.size(), traj.joint_names.size(),
            response->waypoints.size());

        response->success = true;
        response->message = "Plan found: " +
                            std::to_string(traj.points.size()) + " waypoints";
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<ManipulationPlannerNode>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
