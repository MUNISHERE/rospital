#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuRepublisherNode : public rclcpp::Node {
public:
  ImuRepublisherNode() : Node("imu_republisher_node") {
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_ekf", 10);
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/out", 10, std::bind(&ImuRepublisherNode::imuCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "IMU Republisher Node started");
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu) {
    auto new_imu = *imu;  // Sao chép dữ liệu
    new_imu.header.frame_id = "base_footprint_ekf";
    imu_pub_->publish(new_imu);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuRepublisherNode>());
  rclcpp::shutdown();
  return 0;
}