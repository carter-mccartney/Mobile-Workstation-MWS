// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "include/diffdrive_arduino/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_arduino
{
    hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        /* Assigns the parameters read from the 'diffbot.ros2_control.xacro' file
        * to the hardware_parameters vector in 'info_'.
        */
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        /* Copies the name of the left wheel joint that is linked to the 'left_wheel_name'
        * parameter in the 'diffbot.ros2_control.xacro' file.
        */
        cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];

        /* Copies the name of the right wheel joint that is linked to the 'right_wheel_name'
        * parameter in the 'diffbot.ros2_control.xacro' file.
        */
        cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];

        /* Rate at which ros2_control will all read and write functions. */
        cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);

        wheel_l_.setup(cfg_.left_wheel_name);
        wheel_r_.setup(cfg_.right_wheel_name);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> DiffDriveArduinoHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

        /* Tell the hardware interface that the name of the right wheel is held in
        * the variable in 'wheel_r_.name' and that the variable in which
        * the controller should read the velocity is 'wheel_r_.cmd'
        * and that the variable in which to the controller should read the
        * position of the wheel is in 'wheel_r_.pos'.
        */
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> DiffDriveArduinoHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        /* Tell the hardware interface that the name of the left wheel is held in
        * the variable in 'wheel_l_.name' and that the variable in which
        * the controller should set the velocity is 'wheel_l_.cmd'.
        */
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

        /* Tell the hardware interface that the name of the right wheel is held in
        * the variable in 'wheel_r_.name' and that the variable in which
        * the controller should set the velocity is 'wheel_r_.cmd'.
        */
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

        return command_interfaces;
    }

    hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Configuring ...please wait...");
        if (!comms_.connect())
        {
            RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "ERROR: Failed to connect to the Arduino microcontroller.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully connected to the Arduino microcontroller");
        }
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cleaning up ...please wait...");

        if (comms_.connected())
        {
            comms_.disconnect();
        }

        RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully cleaned up!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Activating ...please wait...");
        
        if (!comms_.connected())
        {
            RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "ERROR: Arduino microcontroller not connected in 'on_activate' callback in hardware interface.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Deactivating ...please wait...");
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type DiffDriveArduinoHardware::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        if (!comms_.connected())
        {
            return hardware_interface::return_type::ERROR;
        }

        /* Receive current motor velocity values from the Arduino. */
        if (comms_.read_motor_velocities(&wheel_l_.vel, &wheel_r_.vel) != 0)
        {
            RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "ERROR: Did not receive valid motor velocity values from arduino microcontroller.");
        }

        /* Receive motor tachometer values from the Arduino. */
        if (comms_.read_motor_tachometers(&wheel_l_.tach, &wheel_r_.tach) != 0)
        {
            RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "ERROR: Did not receive valid motor tachometer values from arduino microcontroller.");
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type diffdrive_arduino ::DiffDriveArduinoHardware::write(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        if (!comms_.connected())
        {
            RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "ERROR: Arduino microcontroller not connected in 'write' callback in hardware interface.");
            return hardware_interface::return_type::ERROR;
        }

        float leftVelocity = (float)wheel_l_.cmd;
        float rightVelocity = (float)wheel_r_.cmd;

        comms_.set_motor_values(leftVelocity, rightVelocity);
        return hardware_interface::return_type::OK;
    }

} // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    diffdrive_arduino::DiffDriveArduinoHardware, hardware_interface::SystemInterface)
