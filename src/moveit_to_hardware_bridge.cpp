#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "pluginlib/class_list_macros.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MoveItToHardwareBridge : public hardware_interface::SystemInterface {
public:
	CallbackReturn on_init(const hardware_interface::HardwareInfo &/*info*/) override {
		// if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        //     return CallbackReturn::ERROR;
        // }

		RCLCPP_INFO(rclcpp::get_logger("MoveItToHardwareBridge"), "Initializing hardware interface");

		// Initialize joint vectors
		size_t n_joints = info_.joints.size();
		positions.resize(n_joints, 0.0);
		velocities.resize(n_joints, 0.0);
		commands.resize(n_joints, 0.0);

		return CallbackReturn::SUCCESS;
	}

	CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override {
        RCLCPP_INFO(rclcpp::get_logger("MoveItToHardwareBridge"), "Configuring hardware interface");
        
        // Create a node for publishing
        node = rclcpp::Node::make_shared("moveit_to_hardware_bridge_node");
        pose_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/hardware/arm/goal_pose", 10);
        
        return CallbackReturn::SUCCESS;
    }

	std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
	    std::vector<hardware_interface::StateInterface> state_interfaces;
	    
	    for (size_t i = 0; i < info_.joints.size(); ++i) {
	      state_interfaces.emplace_back(info_.joints[i].name,
	                                    hardware_interface::HW_IF_POSITION,
	                                    &positions[i]);
	      state_interfaces.emplace_back(info_.joints[i].name,
	                                    hardware_interface::HW_IF_VELOCITY,
	                                    &velocities[i]);
	    }

	    return state_interfaces;
	}

	std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
		std::vector<hardware_interface::CommandInterface> command_interfaces;
		
		for (size_t i = 0; i < info_.joints.size(); ++i) {
		  command_interfaces.emplace_back(info_.joints[i].name,
		                                  hardware_interface::HW_IF_POSITION,
		                                  &commands[i]);
		}

		return command_interfaces;
	}

	hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
	{
		// Read joint states from hardware or sensors here
		// For demo, we just mirror commands
		for (size_t i = 0; i < positions.size(); ++i)
			positions[i] = commands[i];
		
		return hardware_interface::return_type::OK;
	}

	hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
	{
		// Send commands to hardware here (e.g., via Dynamixel SDK)
		// Also publish for monitoring
		if (pose_pub) {
			std_msgs::msg::Float64MultiArray msg;
			rclcpp::Rate loop_rate(40); // Hz

			// Print debug info_
			RCLCPP_INFO(rclcpp::get_logger("MoveItToHardwareBridge"), "Publishing command point:");
			for (size_t i = 0; i < commands.size(); ++i) {
				msg.data = commands;
				pose_pub->publish(msg);
			  	RCLCPP_INFO(rclcpp::get_logger("MoveItToHardwareBridge"), "  Joint[%zu] = %.3f", i, commands[i]);
			  	loop_rate.sleep();
			}
		}

		return hardware_interface::return_type::OK;
	}

private:
	std::vector<double> positions, velocities, commands;
	rclcpp::Node::SharedPtr node;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pose_pub;
};

PLUGINLIB_EXPORT_CLASS(MoveItToHardwareBridge, hardware_interface::SystemInterface)