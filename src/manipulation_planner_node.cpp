#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "manipulation_planner_interfaces/srv/plan_trajectory.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using PlanTrajectory = manipulation_planner_interfaces::srv::PlanTrajectory;

class ManipulationPlannerNode : public rclcpp::Node
{
public:
    ManipulationPlannerNode() : Node("manipulation_planner_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing manipulation_planner_node...");

        srv_ = this->create_service<PlanTrajectory>(
            "plan_trajectory",
            std::bind(&ManipulationPlannerNode::plan_trajectory, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Manipulation planner ready");
    }

private:
    rclcpp::Service<PlanTrajectory>::SharedPtr srv_;

    void plan_trajectory(
        const std::shared_ptr<PlanTrajectory::Request> request,
        std::shared_ptr<PlanTrajectory::Response> response)
    {
        const std::string& group = request->planning_group.empty()
            ? "right_arm_group"
            : request->planning_group;

        double planning_time = request->planning_time > 0.0
            ? request->planning_time
            : 5.0;

        RCLCPP_INFO(this->get_logger(),
            "Planning for group '%s', timeout %.1fs",
            group.c_str(), planning_time);

        // MoveGroupInterface necesita el nodo como shared_ptr
        moveit::planning_interface::MoveGroupInterface move_group(
            shared_from_this(), group);

        move_group.setPlanningTime(planning_time);
        move_group.setNumPlanningAttempts(3);
        move_group.setPoseTarget(request->target_pose);

        // Planear sin ejecutar
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto result = move_group.plan(plan);

        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            response->success = false;

            switch (result.val)
            {
                case moveit::core::MoveItErrorCode::TIMED_OUT:
                    response->message = "Planning timed out — no valid plan found";
                    break;
                case moveit::core::MoveItErrorCode::NO_IK_SOLUTION:
                    response->message = "No IK solution found for the given pose";
                    break;
                case moveit::core::MoveItErrorCode::GOAL_IN_COLLISION:
                    response->message = "Goal pose is in collision";
                    break;
                case moveit::core::MoveItErrorCode::START_STATE_IN_COLLISION:
                    response->message = "Start state is in collision";
                    break;
                case moveit::core::MoveItErrorCode::INVALID_GROUP_NAME:
                    response->message = "Invalid planning group: '" + group + "'";
                    break;
                case moveit::core::MoveItErrorCode::INVALID_GOAL_CONSTRAINTS:
                    response->message = "Invalid goal constraints — check target pose";
                    break;
                default:
                    response->message = "Planning failed with MoveIt error code: "
                        + std::to_string(result.val);
                    break;
            }

            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        // Extraer waypoints de la trayectoria
        const auto& trajectory = plan.trajectory_.joint_trajectory;
        const size_t n_joints = trajectory.joint_names.size();
        const size_t n_points = trajectory.points.size();

        if (n_points == 0 || n_joints == 0)
        {
            response->success = false;
            response->message = "Plan succeeded but trajectory is empty";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        response->joint_names = trajectory.joint_names;
        response->waypoints.reserve(n_joints * n_points);
        response->timestamps.reserve(n_points);

        for (const auto& point : trajectory.points)
        {
            for (double pos : point.positions)
                response->waypoints.push_back(pos);

            response->timestamps.push_back(
                point.time_from_start.sec +
                point.time_from_start.nanosec * 1e-9);
        }

        response->success = true;
        response->message = "Plan found: "
            + std::to_string(n_points) + " waypoints, "
            + std::to_string(n_joints) + " joints";

        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
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
