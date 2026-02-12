#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "manipulation_planner_interfaces/srv/solve_right_arm_ik.hpp"
#include "manipulation_planner_interfaces/srv/plan_trajectory.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class TrajectoryPlanner : public rclcpp::Node
{
public:
  TrajectoryPlanner()
    : Node("trajectory_planner")
    {
      using PlanTrajectory = manipulation_planner_interfaces::srv::PlanTrajectory;
      using SolveIK = manipulation_planner_interfaces::srv::SolveRightArmIK;

      // Create service with callback group for avoiding deadlock
      service_cb_group = this->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
      client_cb_group = this->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);

      // Service Server
      srv = this->create_service<PlanTrajectory>(
        "plan_arm_trajectory",
        std::bind(&TrajectoryPlanner::planArmTrajectory, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_cb_group);

      // Service Client
      iksolver_client = this->create_client<SolveIK>(
        "/manipulation/manipulation_planner/solve_right_arm_ik",
        rmw_qos_profile_services_default,
        client_cb_group);

      // Topic Publisher
      position_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/hardware/arms/goal_pose", 10);

      waypoints[0].position.x = 0.0;
      waypoints[0].position.y = 0.0;
      waypoints[0].position.z = 0.8;
      waypoints[0].orientation.w = 1.0;

      waypoints[1].position.x = 0.25;
      waypoints[1].position.y = 0.0;
      waypoints[1].position.z = 1.0;
      waypoints[1].orientation.w = 1.0;

      waypoints[2].position.x = 0.4;
      waypoints[2].position.y = 0.0;
      waypoints[2].position.z = 1.0;
      waypoints[2].orientation.w = 1.0;

      RCLCPP_INFO(this->get_logger(), "Trajectory plan node READY");
    }

private:
  void planArmTrajectory(
    const std::shared_ptr<manipulation_planner_interfaces::srv::PlanTrajectory::Request> req,
    std::shared_ptr<manipulation_planner_interfaces::srv::PlanTrajectory::Response> res)
  {
    using SolveIK = manipulation_planner_interfaces::srv::SolveRightArmIK;
    
    waypoints[3] = req->target_pose.pose;

    // Process each waypoint
    for (size_t i = 0; i < waypoints.size(); i++) {
      // Wait for service to be available
      if (!iksolver_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "IK solver service not available");
        res->success = false;
        return;
      }
      
      auto iksolver_req = std::make_shared<SolveIK::Request>();
      iksolver_req->target_pose.pose = waypoints[i];
      
      // Synchronous call (requires MultiThreadedExecutor or callback groups)
      auto future = iksolver_client->async_send_request(iksolver_req);
      
      auto status = future.wait_for(std::chrono::seconds(5));

      // Wait for result
      if (status == std::future_status::ready)
      {
        auto result = future.get();
        if (result->success) {
          auto message = std_msgs::msg::Float64MultiArray();
          message.data = result->joint_positions;
          position_pub->publish(message);
          
          RCLCPP_INFO(this->get_logger(), "Published waypoint %zu", i);
        } else {
          RCLCPP_ERROR(this->get_logger(), "IK solver failed for waypoint %zu", i);
          res->success = false;
          return;
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call IK service for waypoint %zu", i);
        res->success = false;
        return;
      }
    }
    
    res->success = true;
  }

  rclcpp::Service<manipulation_planner_interfaces::srv::PlanTrajectory>::SharedPtr srv;
  rclcpp::Client<manipulation_planner_interfaces::srv::SolveRightArmIK>::SharedPtr iksolver_client;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub;
  rclcpp::CallbackGroup::SharedPtr service_cb_group;
  rclcpp::CallbackGroup::SharedPtr client_cb_group;

  std::array<geometry_msgs::msg::Pose, 4> waypoints; 
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<TrajectoryPlanner>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}