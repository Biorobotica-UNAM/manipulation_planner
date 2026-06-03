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

        client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        srv_cb_group_    = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/manipulation/joint_states", 10,
            std::bind(&ManipulationPlannerNode::jointStateCallback, this, _1));

        plan_client_ = this->create_client<GetMotionPlan>(
            "/manipulation/plan_kinematic_path",
            rmw_qos_profile_services_default, client_cb_group_);

        srv_ = this->create_service<PlanTrajectory>(
            "plan_trajectory",
            std::bind(&ManipulationPlannerNode::planTrajectory, this, _1, _2),
            rmw_qos_profile_services_default, srv_cb_group_);

        RCLCPP_INFO(this->get_logger(), "Manipulation planner ready");
    }

private:
    rclcpp::Service<PlanTrajectory>::SharedPtr                srv_;
    rclcpp::Client<GetMotionPlan>::SharedPtr                  plan_client_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::CallbackGroup::SharedPtr                          client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr                          srv_cb_group_;

    sensor_msgs::msg::JointState last_joint_state_;
    bool joint_state_received_ = false;

    std::string tipLink(const std::string& group, const std::string& requested_tip) const
	{
		static const std::unordered_map<std::string, std::unordered_set<std::string>> valid_tips = {
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

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        last_joint_state_    = *msg;
        joint_state_received_ = true;
    }

    void planTrajectory(
        const std::shared_ptr<PlanTrajectory::Request>  request,
        std::shared_ptr<PlanTrajectory::Response>       response)
    {
        const std::string& group = request->planning_group.empty()
            ? "right_arm_group" : request->planning_group;

        RCLCPP_INFO(this->get_logger(),
            "Planning request received — group: %s", group.c_str());

        if (!joint_state_received_)
        {
            response->success = false;
            response->message = "No joint_states received yet";
            return;
        }

        if (!plan_client_->wait_for_service(std::chrono::seconds(2)))
        {
            response->success = false;
            response->message = "MoveIt planning service unavailable";
            return;
        }

		const std::string tip = tipLink(group, request->tip_link);
		if (tip.empty())
		{
			response->success = false;
			response->message = "Invalid tip_link '" + request->tip_link + "' for group '" + group + "'";
			return;
		}

        auto srv_req = std::make_shared<GetMotionPlan::Request>();
        auto& req    = srv_req->motion_plan_request;

        req.group_name            = group;
        req.allowed_planning_time = request->planning_time > 0.0 ? request->planning_time : 5.0;
        req.num_planning_attempts = 3;
        req.start_state.joint_state = last_joint_state_;

        req.workspace_parameters.header.frame_id = "base_link";
        req.workspace_parameters.min_corner.x = -2.0;
		req.workspace_parameters.min_corner.y = -2.0;
		req.workspace_parameters.min_corner.z = -2.0;
		req.workspace_parameters.max_corner.x = 2.0;
		req.workspace_parameters.max_corner.y = 2.0;
		req.workspace_parameters.max_corner.z = 2.0;

        // Position constraint
        moveit_msgs::msg::PositionConstraint pos;
        pos.header    = request->target_pose.header;
        pos.link_name = tip;
        pos.weight    = 1.0;

        shape_msgs::msg::SolidPrimitive sphere;
        sphere.type       = shape_msgs::msg::SolidPrimitive::SPHERE;
        sphere.dimensions = {0.01};
        pos.constraint_region.primitives.push_back(sphere);
        pos.constraint_region.primitive_poses.push_back(request->target_pose.pose);

        // Orientation constraint
        moveit_msgs::msg::OrientationConstraint ori;
        ori.header                    = request->target_pose.header;
        ori.link_name                 = tip;
        ori.orientation               = request->target_pose.pose.orientation;
        ori.absolute_x_axis_tolerance = 0.05;
        ori.absolute_y_axis_tolerance = 0.05;
        ori.absolute_z_axis_tolerance = 0.05;
        ori.weight                    = 1.0;

        moveit_msgs::msg::Constraints goal;
        goal.position_constraints.push_back(pos);
        goal.orientation_constraints.push_back(ori);
        req.goal_constraints.push_back(goal);

        std::mutex mtx;
        std::condition_variable cv;
        bool done = false;
        moveit_msgs::srv::GetMotionPlan::Response::SharedPtr result;

        plan_client_->async_send_request(srv_req,
            [&](rclcpp::Client<GetMotionPlan>::SharedFuture future) {
                result = future.get();
                done   = true;
                cv.notify_one();
            });

        std::unique_lock<std::mutex> lock(mtx);
        if (!cv.wait_for(lock,
                std::chrono::seconds((int)req.allowed_planning_time + 1),
                [&]{ return done; }))
        {
            response->success = false;
            response->message = "Planning timeout";
            RCLCPP_ERROR(this->get_logger(), "Planning timeout");
            return;
        }

        if (!result ||
            result->motion_plan_response.error_code.val !=
            moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            response->success = false;
            response->message = result
                ? "Planning failed with error code: " +
                  std::to_string(result->motion_plan_response.error_code.val)
                : "Null response from planner";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        const auto& traj = result->motion_plan_response.trajectory.joint_trajectory;

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
        {
            for (double pos : p.positions)
                response->waypoints.push_back(pos);
        }

		size_t num_joints = traj.joint_names.size();
		size_t num_points = traj.points.size();

		RCLCPP_INFO(this->get_logger(),
			"Flattened trajectory: %zu points × %zu joints = %zu values",
			num_points, num_joints, response->waypoints.size());

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
