#include "diff_robot_hardware/diff_drive_robot.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace diff_drive_robot
{
    hardware_interface::CallbackReturn DiffRobotHardware::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        left_wheel_name = info_.hardware_parameters["left_wheel_name"];
        right_wheel_name = info_.hardware_parameters["right_wheel_name"];
        loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
        device = info_.hardware_parameters["device"];
        baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
        timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
        enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);



        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                rclcpp::get_logger("DiffRobotHardware"),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                rclcpp::get_logger("DiffRobotHardware"),
                "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                rclcpp::get_logger("DiffRobotHardware"),
                "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                rclcpp::get_logger("DiffRobotHardware"),
                "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                rclcpp::get_logger("DiffRobotHardware"),
                "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffRobotHardware::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("DiffRobotHardware"), "Activating ...please wait...");
        comms_.connect(device, baud_rate, timeout_ms);
        RCLCPP_INFO(rclcpp::get_logger("DiffRobotHardware"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffRobotHardware::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("DiffRobotHardware"), "Deactivating ...please wait...");
        comms_.disconnect();
        RCLCPP_INFO(rclcpp::get_logger("DiffRobotHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> DiffRobotHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> DiffRobotHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        return command_interfaces;
    }

    hardware_interface::return_type DiffRobotHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
    {

        //comms_.read_encoder_values();

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type DiffRobotHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
    {

        //comms_.set_motor_values();

        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diff_drive_robot::DiffRobotHardware, hardware_interface::SystemInterface)