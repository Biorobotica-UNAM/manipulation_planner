#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit_msgs/srv/get_motion_plan.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>

#include "manipulation_planner_interfaces/srv/plan_trajectory.hpp"

#include <mutex>
#include <condition_variable>

using std::placeholders::_1;
using std::placeholders::_2;

using PlanTrajectory = manipulation_planner_interfaces::srv::PlanTrajectory;
using GetMotionPlan = moveit_msgs::srv::GetMotionPlan;

class ManipulationPlannerNode : public rclcpp::Node
{
public:
    ManipulationPlannerNode()
    : Node("manipulation_planner_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing manipulation_planner_node...");

        // 🔹 Callback groups
        client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        srv_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // 🔹 Subscriber
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/hardware/joint_states",
            10,
            std::bind(&ManipulationPlannerNode::jointStateCallback, this, _1)
        );

        // 🔹 Cliente MoveIt (sin deprecated)
        plan_client_ = this->create_client<GetMotionPlan>(
            "/hardware/plan_kinematic_path",
            rclcpp::ServicesQoS(),
            client_cb_group_
        );

        // 🔹 Servicio propio
        srv_ = this->create_service<PlanTrajectory>(
            "plan_trajectory",
            std::bind(&ManipulationPlannerNode::planTrajectory, this, _1, _2),
            rclcpp::ServicesQoS(),
            srv_cb_group_
        );

        RCLCPP_INFO(this->get_logger(), "Manipulation planner ready");
    }

private:

    // ROS
    rclcpp::Service<PlanTrajectory>::SharedPtr srv_;
    rclcpp::Client<GetMotionPlan>::SharedPtr plan_client_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr srv_cb_group_;

    // Estado
    sensor_msgs::msg::JointState last_joint_state_;
    bool joint_state_received_ = false;

    // ===============================
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        last_joint_state_ = *msg;
        joint_state_received_ = true;
    }

    // ===============================
    void planTrajectory(
        const std::shared_ptr<PlanTrajectory::Request> request,
        std::shared_ptr<PlanTrajectory::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Planning request received");

        if (!joint_state_received_)
        {
            response->success = false;
            response->message = "No joint_states yet";
            return;
        }

        if (!plan_client_->wait_for_service(std::chrono::seconds(2)))
        {
            response->success = false;
            response->message = "MoveIt service unavailable";
            return;
        }

        // ===============================
        // Construir request MoveIt
        // ===============================
        auto srv_req = std::make_shared<GetMotionPlan::Request>();
        auto &req = srv_req->motion_plan_request;

        req.group_name = request->planning_group;
        req.allowed_planning_time = request->planning_time;
        req.num_planning_attempts = 3;

        req.start_state.joint_state = last_joint_state_;

        // Workspace
        req.workspace_parameters.header.frame_id = "base_link";
        req.workspace_parameters.min_corner.x = -2.0;
        req.workspace_parameters.min_corner.y = -2.0;
        req.workspace_parameters.min_corner.z = -2.0;
        req.workspace_parameters.max_corner.x = 2.0;
        req.workspace_parameters.max_corner.y = 2.0;
        req.workspace_parameters.max_corner.z = 2.0;

        // ===============================
        // Constraints
        // ===============================
        moveit_msgs::msg::Constraints goal;

        moveit_msgs::msg::PositionConstraint pos;
        pos.header = request->target_pose.header;
        pos.link_name = "r_hand_link";

        shape_msgs::msg::SolidPrimitive sphere;
        sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
        sphere.dimensions = {0.02};

        pos.constraint_region.primitives.push_back(sphere);
        pos.constraint_region.primitive_poses.push_back(request->target_pose.pose);
        pos.weight = 1.0;

        goal.position_constraints.push_back(pos);

        moveit_msgs::msg::OrientationConstraint ori;
        ori.header = request->target_pose.header;
        ori.link_name = "r_hand_link";
        ori.orientation = request->target_pose.pose.orientation;

        ori.absolute_x_axis_tolerance = 0.1;
        ori.absolute_y_axis_tolerance = 0.1;
        ori.absolute_z_axis_tolerance = 0.1;
        ori.weight = 1.0;

        goal.orientation_constraints.push_back(ori);

        req.goal_constraints.push_back(goal);

        // ===============================
        // ASYNC + WAIT (como HRI)
        // ===============================
        std::mutex mtx;
        std::condition_variable cv;
        bool done = false;

        moveit_msgs::srv::GetMotionPlan::Response::SharedPtr result;

        auto callback =
        [&](rclcpp::Client<GetMotionPlan>::SharedFuture future)
        {
            RCLCPP_INFO(this->get_logger(), "MoveIt respondió");

            result = future.get();

            done = true;
            cv.notify_one();
        };

        plan_client_->async_send_request(srv_req, callback);

        std::unique_lock<std::mutex> lock(mtx);
        if (!cv.wait_for(lock,
            std::chrono::seconds((int)request->planning_time + 5),
            [&]{ return done; }))
        {
            response->success = false;
            response->message = "Planning timeout";
            RCLCPP_ERROR(this->get_logger(), "Timeout");
            return;
        }

        // ===============================
        // Procesar resultado
        // ===============================
        if (!result)
        {
            response->success = false;
            response->message = "Null response";
            return;
        }

        auto &res = result->motion_plan_response;

        if (res.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            response->success = false;
            response->message = "Planning failed";
            return;
        }

        const auto &traj = res.trajectory.joint_trajectory;

        RCLCPP_INFO(this->get_logger(), "Trajectory points: %lu", traj.points.size());

        response->joint_names = traj.joint_names;

        for (const auto &p : traj.points)
        {
            for (double pos : p.positions)
                response->waypoints.push_back(pos);

            response->timestamps.push_back(
                p.time_from_start.sec +
                p.time_from_start.nanosec * 1e-9);
        }

        response->success = true;
        response->message = "OK";
    }
};

// ===============================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ManipulationPlannerNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
