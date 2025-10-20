#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <libserial/SerialPort.h>
#include <sstream>
#include <vector>

class ImuReaderNode : public rclcpp::Node {
public:
  ImuReaderNode() : Node("imu_reader_node") {
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/out", 10);

    try {
      serial_port_.Open("/dev/ttyARDUINO");
      serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
      // Timeout không được hỗ trợ trực tiếp, dùng mặc định của libserial
      RCLCPP_INFO(this->get_logger(), "Opened /dev/ttyARDUINO");
    } catch (const std::runtime_error& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
      rclcpp::shutdown();
    }

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&ImuReaderNode::readSerial, this));
  }

  ~ImuReaderNode() {
    if (serial_port_.IsOpen()) {
      serial_port_.Close();
    }
  }

private:
  void readSerial() {
    if (!serial_port_.IsOpen()) return;

    try {
      std::string line;
      serial_port_.ReadLine(line, '\n', 100);  // Đọc một dòng, timeout 100ms
      if (line.empty()) return;

      std::vector<float> values = parseData(line);
      if (values.size() != 6) {
        RCLCPP_WARN(this->get_logger(), "Invalid IMU data: %s", line.c_str());
        return;
      }

      auto imu_msg = sensor_msgs::msg::Imu();
      imu_msg.header.stamp = this->now();
      imu_msg.header.frame_id = "imu_link";

      imu_msg.linear_acceleration.x = values[0] * 9.81;
      imu_msg.linear_acceleration.y = values[1] * 9.81;
      imu_msg.linear_acceleration.z = values[2] * 9.81;
      imu_msg.angular_velocity.x = values[3] * M_PI / 180.0;
      imu_msg.angular_velocity.y = values[4] * M_PI / 180.0;
      imu_msg.angular_velocity.z = values[5] * M_PI / 180.0;

      imu_pub_->publish(imu_msg);
    } catch (const std::runtime_error& e) {
      RCLCPP_ERROR(this->get_logger(), "Serial error: %s", e.what());
    }
  }

  std::vector<float> parseData(const std::string& line) {
    std::vector<float> values;
    std::stringstream ss(line);
    std::string token;

    // Bỏ qua dữ liệu ODrive nếu có (r, l, rp, lp)
    for (int i = 0; i < 4; ++i) {
      std::getline(ss, token, ',');
    }
    std::getline(ss, token, ',');  // rp
    std::getline(ss, token, ',');  // lp

    std::getline(ss, token, ',');  // imu
    if (token != "imu") return values;

    while (std::getline(ss, token, ',')) {
      try {
        values.push_back(std::stof(token));
      } catch (...) {
        values.clear();
        break;
      }
    }
    return values;  // [ax, ay, az, gx, gy, gz]
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  LibSerial::SerialPort serial_port_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuReaderNode>());
  rclcpp::shutdown();
  return 0;
}