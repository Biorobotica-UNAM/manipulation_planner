#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "arms/srv/arm_movement_plan.hpp"

class ArmMovementPlanner : public rclcpp::Node {
public:
	ArmMovementPlanner() : rclcpp::Node("plan_arm_movement_service"),
			move_group(std::shared_ptr<rclcpp::Node>(this, [](auto*){}), "right_arm") {
		move_group.setPlanningTime(10.0);

		plan_arm_movement_service_ = this->create_service<arms::srv::ArmMovementPlan>("plan_arm_movement", std::bind(&ArmMovementPlanner::planArmMovementServer, this, std::placeholders::_1, std::placeholders::_2));

		RCLCPP_INFO(this->get_logger(), "=== Plan Arm Movement Service READY ===");
	}

private:
	bool moveToCartesianPose(double x, double y, double z, double roll, double pitch, double yaw) {
		bool success = false;
		geometry_msgs::msg::Pose target_pose;
		tf2::Quaternion q;
		moveit::planning_interface::MoveGroupInterface::Plan plan;

		target_pose.position.x = x;
		target_pose.position.y = y;
		target_pose.position.z = z;

		q.setRPY(roll, pitch, yaw);
		q.normalize();
		target_pose.orientation.x = q.x();
		target_pose.orientation.y = q.y();
		target_pose.orientation.z = q.z();
		target_pose.orientation.w = q.w();

		move_group.setPoseTarget(target_pose);

		if(static_cast<bool>(move_group.plan(plan))) {
			move_group.move();
			success = true;
		}

		move_group.clearPoseTargets();
		return success;
	}

	bool moveToNamedPose(const std::string &pose_name) {
		bool success = false;
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		
		move_group.setNamedTarget(pose_name);

		if(static_cast<bool>(move_group.plan(plan))) {
			move_group.move();
			success = true;
		}

		return success;
	}

	void planArmMovementServer(std::shared_ptr<arms::srv::ArmMovementPlan::Request> req,
			std::shared_ptr<arms::srv::ArmMovementPlan::Response> res) {
		bool success = false;

		if(!req->pose_name.empty()) {
			RCLCPP_INFO(this->get_logger(), "=== Movement to predefined pose: %s ===", req->pose_name.c_str());
			success = moveToNamedPose(req->pose_name);
			res->success = success;
			res->message = success ? "Predefined pose successfully executed" : "Predefined pose exited with failure";
		} else {
			RCLCPP_INFO(this->get_logger(), "=== Movement to cartesian pose. x=%.2f, y=%.2f, z=%.2f", req->x, req->y, req->z);
			success = moveToCartesianPose(req->x, req->y, req->z, req->roll, req->pitch, req->yaw);
	        res->success = success;
	        res->message = success ? "Cartesian pose successfully executed" : "Cartesian pose exited with failure";
		}
	}

	moveit::planning_interface::MoveGroupInterface move_group;
	rclcpp::Service<arms::srv::ArmMovementPlan>::SharedPtr plan_arm_movement_service_;
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ArmMovementPlanner>();

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}