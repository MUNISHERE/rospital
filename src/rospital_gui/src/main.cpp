#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "rospital_gui/mainwindow.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    
    // Tạo node và thiết lập tham số map
    auto node = rclcpp::Node::make_shared("rospital_gui_node");
    std::string map_path = "/home/rospital/rospital_ws/install/rospital_map/share/rospital_map/map/small_house.yaml";
    if (!node->has_parameter("map")) {
        node->declare_parameter("map", map_path);
        RCLCPP_INFO(node->get_logger(), "Set default map parameter: %s", map_path.c_str());
    } else {
        node->get_parameter("map", map_path);
        RCLCPP_INFO(node->get_logger(), "Map parameter already set: %s", map_path.c_str());
    }

    // Đảm bảo node sẵn sàng
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    rospital_gui::MainWindow window(node); // Truyền node vào constructor
    window.show();
    int result = app.exec();
    rclcpp::shutdown();
    return result;
}