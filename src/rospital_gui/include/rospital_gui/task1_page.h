#ifndef TASK1_PAGE_H
#define TASK1_PAGE_H

#include "rospital_gui/rviz_page.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <rviz_common/tool_manager.hpp>
#include <QWidget>
#include "ui_mainwindow.h"

namespace rospital_gui {

class Task1Page : public RVizPage
{
    Q_OBJECT
public:
    explicit Task1Page(std::shared_ptr<rclcpp::Node> node, 
                       std::shared_ptr<tf2_ros::Buffer> tf_buffer, 
                       Ui::MainWindow *ui, 
                       QWidget *parent = nullptr);
    ~Task1Page();

public slots:
    void onGoalAdded(const geometry_msgs::msg::PoseStamped &pose); // Xử lý tín hiệu goalAdded từ MapPage
    void loadRegionsForTasks(); // Tải danh sách mục tiêu từ file YAML

private slots:
    void on_task1PlayBtn_clicked(); // Gửi mục tiêu khi nhấn nút Play
    void on_task1GoalSelectBtn_clicked(); // Chọn mục tiêu từ danh sách
    void on_task1GoalBtn_clicked(); // Kích hoạt công cụ GoalPose
    void on_cancel1Btn_clicked(); // Hủy nhiệm vụ
    void on_site1Btn_clicked(); // Kích hoạt công cụ InitialPose

private:
    void sendGoal(); // Gửi mục tiêu đến action server
    void populateGoalList(); // Điền danh sách mục tiêu vào task1GoalList
    QString getMapNameFromParam(); // Lấy tên bản đồ từ tham số ROS

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    Ui::MainWindow *ui;
    QString map_dir_path_;
    double current_size_;
    struct Goal {
        QString name;
        geometry_msgs::msg::PoseStamped pose;
        double size;
    };
    std::vector<Goal> task_goals_;
};

} // namespace rospital_gui

#endif // TASK1_PAGE_H