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
      service_cb_group_ = this->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
      client_cb_group_ = this->create_callback_group(
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
      positon_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/hardware/arms/goal_pose", 10);

      waypoints[0].position.x = 0.0;
      waypoints[0].position.y = 0.0;
      waypoints[0].position.z = 0.0;

      waypoints[1].position.x = 0.0;
      waypoints[1].position.y = 0.0;
      waypoints[1].position.z = 0.0;

      waypoints[2].position.x = 0.0;
      waypoints[2].position.y = 0.0;
      waypoints[2].position.z = 0.0;

      RCLCPP_INFO(this->get_logger(), "Trajectory plan node READY");
    }

private:
  void planArmTrajectory(
    const std::shared_ptr<manipulation_planner_interfaces::srv::PlanTrajectory::Request> req,
    std::shared_ptr<manipulation_planner_interfaces::srv::PlanTrajectory::Response> res)
  {
    waypoints[3] = res->target_pose.pose;

    for(int i = 0; i < waypoints.size(); i++)
    {
      auto iksolver_req = std::make_shared<SolveIK::Request>();
      iksolver_req->target_pose = waypoints[i];

      auto future = iksolver_client->async_send_request(
        iksolver_req,
        [this, response](rclcpp::Client<SolveIK>::SharedFuture future)
        {
          try
          {
            auto result = future.get();

            if (result->success)
            {
              auto message = std_msgs::msg::Float64MultiArray();
              message.data = result->joint_positions;
              position_pub->publish(message);

              response->success = true;
            } else {
              RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
              response->success = false;
            }
          }
          catch (const std::exception &e)
          {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
          }
        }
      );
    }
  }

  rclcpp::Service<manipulation_planner_interfaces::srv::PlanTrajectory>::SharedPtr srv;
  rclcpp::Client<manipulation_planner_interfaces::srv::SolveRightArmIK>::SharedPtr iksolver_client;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub;
  rclcpp::CallbackGroup::SharedPtr service_cb_group;
  rclcpp::CallbackGroup::SharedPtr client_cb_group;

  std::array<geometry_msgs::msg::Pose, 4> waypoints; 
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(std::make_shared<TrajectoryPlanner>())
  executor.spin();

  rclcpp::shutdown();
  return 0;
}