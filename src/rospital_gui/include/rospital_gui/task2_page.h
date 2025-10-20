#ifndef TASK2_PAGE_H
#define TASK2_PAGE_H

#include "rospital_gui/rviz_page.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include "ui_mainwindow.h"

namespace rospital_gui {

class Task2Page : public RVizPage
{
    Q_OBJECT
public:
    explicit Task2Page(std::shared_ptr<rclcpp::Node> node, 
                       std::shared_ptr<tf2_ros::Buffer> tf_buffer, 
                       Ui::MainWindow *ui, 
                       QWidget *parent = nullptr);
    ~Task2Page();

public slots:
    void onGoalAdded(const geometry_msgs::msg::PoseStamped &pose);
    void loadRegionsForTasks();

private slots:
    void on_task2PlayBtn_clicked();
    void on_task2GoalSelectBtn_clicked();
    void on_task2GoalBtn_clicked();
    void on_cancel2Btn_clicked();
    void on_site2Btn_clicked();
    void on_removeGoal_clicked(int index);

private:
    void sendMultiGoals();
    void populateGoalList();
    QString getMapNameFromParam();

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    Ui::MainWindow *ui;
    QString map_dir_path_;
    int current_goal_index_;
    struct Goal {
        QString name;
        geometry_msgs::msg::PoseStamped pose;
        double size;
        QLabel* name_label;
        QLabel* distance_label;
        QLabel* time_label;
        QPushButton* remove_button;
    };
    std::vector<Goal> task_goals_;
    std::vector<Goal> selected_goals_;
};

} // namespace rospital_gui

#endif // TASK2_PAGE_H
