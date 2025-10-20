// Copyright 2021 ros2_control Development Team
// Licensed under the Apache License, Version 2.0 (the "License");
// http://www.apache.org/licenses/LICENSE-2.0

#include "rospital_hardware_interface/robot_hardware_interface.hpp"
#include <libserial/SerialPort.h>
#include <iostream>
#include <string>
#include <sstream>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rospital_firmware {

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RospitalInterface::on_init(
    const hardware_interface::HardwareInfo &hardware_info) {
  auto connect = hardware_interface::SystemInterface::on_init(hardware_info);
  if (connect != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
    return connect;
  }

  try {
    port_ = info_.hardware_parameters.at("port");
    RCLCPP_INFO(rclcpp::get_logger("RospitalInterface"), "Using serial port: %s", port_.c_str());
  } catch (const std::out_of_range &e) {
    RCLCPP_FATAL(rclcpp::get_logger("RospitalInterface"), "No Serial Port provided! Aborting");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  velocity_commands_.resize(info_.joints.size(), 0.0); // Khởi tạo mặc định
  position_states_.resize(info_.joints.size(), 0.0);
  velocity_states_.resize(info_.joints.size(), 0.0);

  last_run_ = rclcpp::Clock().now();
  consecutive_failures_ = 0;

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("RospitalInterface"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("RospitalInterface"),
                   "Joint '%s' has '%s' command interface found. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("RospitalInterface"),
                   "Joint '%s' has %zu state interfaces. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("RospitalInterface"),
                   "Joint '%s' has '%s' as first state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("RospitalInterface"),
                   "Joint '%s' has '%s' as second state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RospitalInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RospitalInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]);
  }
  return command_interfaces;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RospitalInterface::on_activate(
    const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RospitalInterface"), "Starting robot hardware ...");

  try {
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    RCLCPP_INFO(rclcpp::get_logger("RospitalInterface"), "Serial port %s opened successfully", port_.c_str());

    arduino_.FlushIOBuffers();

    std::string init_response;
    auto start = rclcpp::Clock().now();
    bool data_received = false;
    while ((rclcpp::Clock().now() - start).seconds() < 20.0) {
      if (arduino_.IsDataAvailable()) {
        arduino_.ReadLine(init_response, '\n', 500);
        RCLCPP_INFO(rclcpp::get_logger("RospitalInterface"), "Initial response from Arduino: %s", init_response.c_str());
        data_received = true;
        break;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    if (!data_received) {
      RCLCPP_ERROR(rclcpp::get_logger("RospitalInterface"), "No data received from Arduino after 10 seconds. Proceeding with default values...");
    }

  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("RospitalInterface"), "Error opening connection with port %s: %s", port_.c_str(), e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("RospitalInterface"), "Hardware started, ready to take commands");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RospitalInterface::on_deactivate(
    const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RospitalInterface"), "Stopping robot hardware ...");

  if (arduino_.IsOpen()) {
    try {
      arduino_.Write("r0,l0,\n");
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      arduino_.Close();
      RCLCPP_INFO(rclcpp::get_logger("RospitalInterface"), "Serial port %s closed successfully", port_.c_str());
    } catch (const std::exception &e) {
      RCLCPP_FATAL(rclcpp::get_logger("RospitalInterface"), "Error closing connection with port %s: %s", port_.c_str(), e.what());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("RospitalInterface"), "Hardware stopped");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RospitalInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  if (!arduino_.IsOpen()) {
    RCLCPP_ERROR(rclcpp::get_logger("RospitalInterface"), "Serial port is not open");
    return hardware_interface::return_type::ERROR;
  }

  static bool first_read = true;
  std::string message;
  int retry_count = 0;
  const int MAX_RETRIES = 5;
  const int MAX_CONSECUTIVE_FAILURES = 5;

  while (retry_count < MAX_RETRIES) {
    try {
      auto start = rclcpp::Clock().now();
      while (!arduino_.IsDataAvailable() && (rclcpp::Clock().now() - start).seconds() < 2.0) {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
      }

      if (arduino_.IsDataAvailable()) {
        arduino_.ReadLine(message, '\n', 500);
        if (!message.empty()) {
          RCLCPP_DEBUG(rclcpp::get_logger("RospitalInterface"), "Received raw message: %s", message.c_str());
          if (message.find("DEBUG:") != std::string::npos || message.find("ft:") != std::string::npos || 
              message.find("0x") != std::string::npos) {
            RCLCPP_DEBUG(rclcpp::get_logger("RospitalInterface"), "Ignoring debug/CAN message: %s", message.c_str());
            continue;
          }
          break;
        } else {
          RCLCPP_WARN(rclcpp::get_logger("RospitalInterface"), "Received empty message from Arduino, retrying... (Attempt %d/%d)", retry_count + 1, MAX_RETRIES);
        }
      } else {
        RCLCPP_WARN(rclcpp::get_logger("RospitalInterface"), "No data available from Arduino after 2s timeout, retrying... (Attempt %d/%d)", retry_count + 1, MAX_RETRIES);
      }
      retry_count++;
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("RospitalInterface"), "Error reading from Arduino: %s, retrying... (Attempt %d/%d)", e.what(), retry_count + 1, MAX_RETRIES);
      retry_count++;
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }

  if (retry_count >= MAX_RETRIES) {
    consecutive_failures_++;
    RCLCPP_ERROR(rclcpp::get_logger("RospitalInterface"), "Failed to read from Arduino after %d retries. Consecutive failures: %d", MAX_RETRIES, consecutive_failures_);
    if (consecutive_failures_ >= MAX_CONSECUTIVE_FAILURES) {
      RCLCPP_ERROR(rclcpp::get_logger("RospitalInterface"), "Too many consecutive failures (%d). Attempting to reset serial connection...", consecutive_failures_);
      try {
        if (arduino_.IsOpen()) arduino_.Close();
        arduino_.Open(port_);
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        arduino_.FlushIOBuffers();
        RCLCPP_INFO(rclcpp::get_logger("RospitalInterface"), "Serial port %s reopened successfully", port_.c_str());
        consecutive_failures_ = 0;
      } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("RospitalInterface"), "Failed to reopen serial port %s: %s", port_.c_str(), e.what());
        return hardware_interface::return_type::ERROR;
      }
    }
    if (first_read) {
      first_read = false;
      return hardware_interface::return_type::OK;
    }
    return hardware_interface::return_type::ERROR;
  }

  first_read = false;

  // Phân tích tin nhắn
  std::stringstream ss(message);
  std::string token;
  bool updated = false;

  while (std::getline(ss, token, ',')) {
    if (token.empty()) continue;

    RCLCPP_DEBUG(rclcpp::get_logger("RospitalInterface"), "Processing token: %s", token.c_str());

    try {
      size_t pos;
      // Chỉ xử lý token nếu đúng định dạng "r<val>", "l<val>", "rp<val>", "lp<val>"
      if (token.length() < 2) {
        RCLCPP_WARN(rclcpp::get_logger("RospitalInterface"), "Invalid token length: %s", token.c_str());
        continue;
      }

      if (token[0] == 'r' && token[1] != 'p') {
        std::string value = token.substr(1);
        if (!value.empty() && value.find_first_not_of("0123456789.-") == std::string::npos) {
          velocity_states_[0] = std::stod(value, &pos);
          RCLCPP_INFO(rclcpp::get_logger("RospitalInterface"), "Right velocity state updated: %f", velocity_states_[0]);
          updated = true;
        } else {
          RCLCPP_WARN(rclcpp::get_logger("RospitalInterface"), "Invalid right velocity value: %s", value.c_str());
        }
      } else if (token[0] == 'l' && token[1] != 'p') {
        std::string value = token.substr(1);
        if (!value.empty() && value.find_first_not_of("0123456789.-") == std::string::npos) {
          velocity_states_[1] = -std::stod(value, &pos);
          RCLCPP_INFO(rclcpp::get_logger("RospitalInterface"), "Left velocity state updated: %f", velocity_states_[1]);
          updated = true;
        } else {
          RCLCPP_WARN(rclcpp::get_logger("RospitalInterface"), "Invalid left velocity value: %s", value.c_str());
        }
      } else if (token.rfind("rp", 0) == 0) {
        std::string value = token.substr(2);
        if (!value.empty() && value.find_first_not_of("0123456789.-") == std::string::npos) {
          position_states_[0] = std::stod(value, &pos);
          RCLCPP_INFO(rclcpp::get_logger("RospitalInterface"), "Right position state updated: %f", position_states_[0]);
          updated = true;
        } else {
          RCLCPP_WARN(rclcpp::get_logger("RospitalInterface"), "Invalid right position value: %s", value.c_str());
        }
      } else if (token.rfind("lp", 0) == 0) {
        std::string value = token.substr(2);
        if (!value.empty() && value.find_first_not_of("0123456789.-") == std::string::npos) {
          position_states_[1] = -std::stod(value, &pos);
          RCLCPP_INFO(rclcpp::get_logger("RospitalInterface"), "Left position state updated: %f", position_states_[1]);
          updated = true;
        } else {
          RCLCPP_WARN(rclcpp::get_logger("RospitalInterface"), "Invalid left position value: %s", value.c_str());
        }
      } else {
        RCLCPP_WARN(rclcpp::get_logger("RospitalInterface"), "Unknown token: %s", token.c_str());
      }
    } catch (const std::invalid_argument &e) {
      RCLCPP_ERROR(rclcpp::get_logger("RospitalInterface"), "Invalid number format in token: %s", token.c_str());
    } catch (const std::out_of_range &e) {
      RCLCPP_ERROR(rclcpp::get_logger("RospitalInterface"), "Number out of range in token: %s", token.c_str());
    }
  }

  if (updated) {
    last_run_ = rclcpp::Clock().now();
    consecutive_failures_ = 0; // Reset consecutive failures on successful update
    RCLCPP_DEBUG(rclcpp::get_logger("RospitalInterface"), "State updated successfully");
  } else {
    RCLCPP_WARN(rclcpp::get_logger("RospitalInterface"), "No valid state updates in message");
  }

  if ((rclcpp::Clock().now() - last_run_).seconds() > 20.0) {
    RCLCPP_ERROR(rclcpp::get_logger("RospitalInterface"), "No valid updates for 20 seconds. Possible communication failure.");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RospitalInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  if (info_.joints.size() < 2) {
    RCLCPP_ERROR(rclcpp::get_logger("RospitalInterface"), "Not enough joints defined (minimum 2 required)");
    return hardware_interface::return_type::ERROR;
  }

  if (!arduino_.IsOpen()) {
    RCLCPP_ERROR(rclcpp::get_logger("RospitalInterface"), "Serial port is not open");
    return hardware_interface::return_type::ERROR;
  }

  std::stringstream message_stream;
  const double MAX_VELOCITY = 10.0;
  velocity_commands_[0] = std::clamp(velocity_commands_[0], -MAX_VELOCITY, MAX_VELOCITY);
  velocity_commands_[1] = std::clamp(velocity_commands_[1], -MAX_VELOCITY, MAX_VELOCITY);

  float vel0 = velocity_commands_[0] / 2 / M_PI;
  float vel1 = -velocity_commands_[1] / 2 / M_PI;
  message_stream << "r" << vel0 << ",l" << vel1 << ",\n";

  std::string message = message_stream.str();
  RCLCPP_INFO(rclcpp::get_logger("RospitalInterface"), "Sending to Arduino: %s", message.c_str());

  try {
    arduino_.Write(message);
    RCLCPP_DEBUG(rclcpp::get_logger("RospitalInterface"), "Message sent successfully");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("RospitalInterface"), "Error sending message '%s' to port %s: %s",
                 message.c_str(), port_.c_str(), e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace rospital_firmware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rospital_firmware::RospitalInterface, hardware_interface::SystemInterface)