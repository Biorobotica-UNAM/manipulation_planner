#include <memory>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "manipulation_planner_interfaces/srv/solve_right_arm_ik.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class RightArmIKSolver : public rclcpp::Node
{
public:
  using SolveIK =
    manipulation_planner_interfaces::srv::SolveRightArmIK;

  RightArmIKSolver()
  : Node("right_arm_ik_solver")
  {
    srv_ = this->create_service<SolveIK>(
      "/manipulation_planner/solve_right_arm_ik",
      std::bind(&RightArmIKSolver::solveIK, this, _1, _2));

    RCLCPP_INFO(this->get_logger(),
      "Right arm IK solver ready");
  }

private:
  rclcpp::Service<SolveIK>::SharedPtr srv_;

  static constexpr double L1 = 0.216;
  static constexpr double L2 = 0.37;
  static constexpr double SHOULDER_Z = 1.07;

  void solveIK(
    const std::shared_ptr<SolveIK::Request> req,
    std::shared_ptr<SolveIK::Response> res)
  {
    const auto &p = req->target_pose.pose.position;

    double dx = std::sqrt(p.x*p.x + p.y*p.y);
    double dz = p.z - SHOULDER_Z;
    double r  = std::sqrt(dx*dx + dz*dz);

    if (r > L1 + L2 || r < std::fabs(L1 - L2))
    {
      res->success = false;
      res->message = "Target out of reach";
      return;
    }

    double q_elbow =
      std::acos((r*r - L1*L1 - L2*L2) / (2*L1*L2));

    double phi = std::atan2(dx, -dz);
    double psi = std::atan2(
      L2 * std::sin(q_elbow),
      L1 + L2 * std::cos(q_elbow));

    res->joint_positions = {
      phi - psi,
      std::atan2(p.y, p.x),
      q_elbow,
      0.0
    };

    res->success = true;
    res->message = "IK solved";
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RightArmIKSolver>());
  rclcpp::shutdown();
  return 0;
}

