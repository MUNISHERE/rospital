// Copyright 2021 ros2_control Development Team
// Licensed under the Apache License, Version 2.0 (the "License");
// http://www.apache.org/licenses/LICENSE-2.0

#ifndef ROSPITAL_HARDWARE_INTERFACE__ROBOT_HARDWARE_INTERFACE_HPP_
#define ROSPITAL_HARDWARE_INTERFACE__ROBOT_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <libserial/SerialPort.h>

namespace rospital_firmware {

class RospitalInterface : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RospitalInterface)

  /// Khởi tạo giao diện phần cứng
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo & info) override;

  /// Xuất các giao diện trạng thái (position, velocity)
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /// Xuất các giao diện lệnh (velocity)
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /// Kích hoạt giao diện phần cứng
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;

  /// Tắt giao diện phần cứng
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & previous_state) override;

  /// Đọc dữ liệu từ Arduino qua Serial
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Gửi lệnh vận tốc tới Arduino qua Serial
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  LibSerial::SerialPort arduino_;
  std::string port_;
  std::vector<double> velocity_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  rclcpp::Time last_run_;
  int consecutive_failures_;
};

}  // namespace rospital_firmware

#endif  // ROSPITAL_HARDWARE_INTERFACE__ROBOT_HARDWARE_INTERFACE_HPP_