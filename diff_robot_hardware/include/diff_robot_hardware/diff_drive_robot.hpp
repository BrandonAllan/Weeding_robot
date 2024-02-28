#ifndef DIFF_DRIVE_ROBOT_HPP_
#define DIFF_DRIVE_ROBOT_HPP_

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "visibility_control.h"

namespace diff_drive_robot
{
class DiffRobotHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DiffRobotHardware)
    
    DIFF_DRIVE_ROBOT_PUBLIC
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    DIFF_DRIVE_ROBOT_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    DIFF_DRIVE_ROBOT_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    DIFF_DRIVE_ROBOT_PUBLIC
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    DIFF_DRIVE_ROBOT_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    
    DIFF_DRIVE_ROBOT_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    DIFF_DRIVE_ROBOT_PUBLIC
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    std::vector<double> hw_commands_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
};
}
#endif 
