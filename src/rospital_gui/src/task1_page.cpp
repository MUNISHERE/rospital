#include "rospital_gui/task1_page.h"
#include <QDir>
#include <QFileInfo>
#include <QVBoxLayout>
#include <QMessageBox>
#include <rviz_default_plugins/displays/map/map_display.hpp>
#include <rviz_default_plugins/displays/robot_model/robot_model_display.hpp>
#include <rviz_default_plugins/displays/tf/tf_display.hpp>
#include <rviz_default_plugins/displays/marker/marker_display.hpp>
#include <rviz_default_plugins/tools/pose_estimate/initial_pose_tool.hpp>
#include <rviz_default_plugins/tools/goal_pose/goal_tool.hpp>

namespace rospital_gui {

Task1Page::Task1Page(std::shared_ptr<rclcpp::Node> node, 
                     std::shared_ptr<tf2_ros::Buffer> tf_buffer, 
                     Ui::MainWindow *ui, 
                     QWidget *parent)
    : RVizPage(node, tf_buffer, ui->map1Wid, parent),
      node_(node),
      tf_buffer_(tf_buffer),
      ui(ui),
      map_dir_path_(QDir::homePath() + "/rospital/rospital_ws/src/rospital_map/map"),
      current_size_(0.5),
      task_goals_()
{
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "/navigate_to_pose");
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    initial_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

    goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            QString goal_name = QString("Goal_%1").arg(task_goals_.size() + 1);
            QString goal_str = QString("%1 (%2, %3, %4)").arg(goal_name)
                              .arg(msg->pose.position.x, 0, 'f', 2)
                              .arg(msg->pose.position.y, 0, 'f', 2)
                              .arg(0.5, 0, 'f', 2);
            this->ui->task1GoalList->addItem(goal_str);
            task_goals_.push_back({goal_name, *msg, 0.5});
            QMessageBox::information(this, "Thông báo", "Đã thêm mục tiêu: " + goal_name);
        });

    connect(ui->task1PlayBtn, &QPushButton::clicked, this, &Task1Page::on_task1PlayBtn_clicked);
    connect(ui->task1GoalSelectBtn, &QPushButton::clicked, this, &Task1Page::on_task1GoalSelectBtn_clicked);
    connect(ui->task1GoalBtn, &QPushButton::clicked, this, &Task1Page::on_task1GoalBtn_clicked);
    connect(ui->cancel1Btn, &QPushButton::clicked, this, &Task1Page::on_cancel1Btn_clicked);
    connect(ui->site1Btn, &QPushButton::clicked, this, &Task1Page::on_site1Btn_clicked);

    ui->distanceStatus1->setText("");
    ui->timeLastStatus1->setText("0 s");

    populateGoalList();
}

Task1Page::~Task1Page() {}

QString Task1Page::getMapNameFromParam()
{
    std::string map_path;
    node_->get_parameter("map", map_path);
    QFileInfo file_info(QString::fromStdString(map_path));
    return file_info.baseName();
}

void Task1Page::loadRegionsForTasks()
{
    populateGoalList();
}

void Task1Page::onGoalAdded(const geometry_msgs::msg::PoseStamped &pose)
{
    QString goal_name = QString("Goal_%1").arg(task_goals_.size() + 1);
    QString goal_str = QString("%1 (%2, %3, %4)").arg(goal_name)
                      .arg(pose.pose.position.x, 0, 'f', 2)
                      .arg(pose.pose.position.y, 0, 'f', 2)
                      .arg(0.5, 0, 'f', 2);
    ui->task1GoalList->addItem(goal_str);
    task_goals_.push_back({goal_name, pose, 0.5});
    QMessageBox::information(this, "Thông báo", "Đã thêm mục tiêu: " + goal_name);
}

void Task1Page::on_task1PlayBtn_clicked()
{
    sendGoal();
}

void Task1Page::on_task1GoalSelectBtn_clicked()
{
    int index = ui->task1GoalList->currentIndex();
    if (index >= 0 && index < static_cast<int>(task_goals_.size())) {
        QMessageBox::information(this, "Thông báo", "Đã chọn mục tiêu: " + task_goals_[index].name);
    } else {
        QMessageBox::warning(this, "Lỗi", "Vui lòng chọn một mục tiêu từ danh sách.");
    }
}

void Task1Page::on_task1GoalBtn_clicked()
{
    auto tool_manager = manager_->getToolManager();
    tool_manager->setCurrentTool(tool_manager->getTool(1)); // GoalPose tool
    QMessageBox::information(this, "Thông báo", "Chọn mục tiêu trên bản đồ");
}

void Task1Page::on_cancel1Btn_clicked()
{
    nav_client_->async_cancel_all_goals();
    ui->distanceStatus1->setText("");
    ui->timeLastStatus1->setText("0 s");
    QMessageBox::information(this, "Thông báo", "Đã hủy nhiệm vụ");
}

void Task1Page::on_site1Btn_clicked()
{
    auto tool_manager = manager_->getToolManager();
    tool_manager->setCurrentTool(tool_manager->getTool(0)); // InitialPose tool
    QMessageBox::information(this, "Thông báo", "Chọn vị trí bắt đầu trên bản đồ");
}

void Task1Page::sendGoal()
{
    int index = ui->task1GoalList->currentIndex();
    if (index < 0 || index >= static_cast<int>(task_goals_.size())) {
        ui->distanceStatus1->setText("");
        ui->timeLastStatus1->setText("0 s");
        QMessageBox::warning(this, "Lỗi", "Vui lòng chọn một mục tiêu từ danh sách.");
        return;
    }

    if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
        ui->distanceStatus1->setText("");
        ui->timeLastStatus1->setText("0 s");
        QMessageBox::critical(this, "Lỗi", "Không thể kết nối với /navigate_to_pose action server.");
        return;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = task_goals_[index].pose;
    goal_msg.pose.header.stamp = node_->get_clock()->now();

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.feedback_callback = 
        [this](auto, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) {
            double distance = feedback->distance_remaining;
            double time_remaining = feedback->estimated_time_remaining.sec + 
                                   feedback->estimated_time_remaining.nanosec / 1e9;
            this->ui->distanceStatus1->setText(QString("%1 m").arg(distance, 0, 'f', 2));
            this->ui->timeLastStatus1->setText(QString("%1 s").arg(time_remaining, 0, 'f', 2));
        };
    send_goal_options.result_callback = 
        [this](const auto &result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                this->ui->distanceStatus1->setText("0 m");
                this->ui->timeLastStatus1->setText("0 s");
                QMessageBox::information(this, "Thông báo", "Đã đến mục tiêu: " + task_goals_[this->ui->task1GoalList->currentIndex()].name);
            } else {
                this->ui->distanceStatus1->setText("");
                this->ui->timeLastStatus1->setText("0 s");
                QMessageBox::critical(this, "Lỗi", "Không thể hoàn thành nhiệm vụ.");
            }
        };

    nav_client_->async_send_goal(goal_msg, send_goal_options);
}

void Task1Page::populateGoalList()
{
    ui->task1GoalList->clear();
    task_goals_.clear();
    QDir dir(map_dir_path_);
    QStringList regions = dir.entryList(QStringList() << "*.yaml", QDir::Files);
    for (const QString &region : regions) {
        QString yaml_file = map_dir_path_ + "/" + region;
        try {
            YAML::Node config = YAML::LoadFile(yaml_file.toStdString());
            for (const auto &item : config["regions"]) {
                QString name = QString::fromStdString(item["name"].as<std::string>());
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.header.stamp = node_->get_clock()->now();
                pose.pose.position.x = item["pose"]["position"]["x"].as<double>();
                pose.pose.position.y = item["pose"]["position"]["y"].as<double>();
                pose.pose.position.z = item["pose"]["position"]["z"].as<double>();
                pose.pose.orientation.x = item["pose"]["orientation"]["x"].as<double>();
                pose.pose.orientation.y = item["pose"]["orientation"]["y"].as<double>();
                pose.pose.orientation.z = item["pose"]["orientation"]["z"].as<double>();
                pose.pose.orientation.w = item["pose"]["orientation"]["w"].as<double>();
                double size = item["size"].as<double>();
                QString goal_str = QString("%1 (%2, %3, %4)").arg(name)
                                  .arg(pose.pose.position.x, 0, 'f', 2)
                                  .arg(pose.pose.position.y, 0, 'f', 2)
                                  .arg(size, 0, 'f', 2);
                ui->task1GoalList->addItem(goal_str);
                task_goals_.push_back({name, pose, size});
            }
            QMessageBox::information(this, "Thông báo", "Đã tải danh sách mục tiêu từ: " + yaml_file);
        } catch (const YAML::Exception &e) {
            RCLCPP_WARN(node_->get_logger(), "Failed to load regions from %s: %s", 
                        yaml_file.toStdString().c_str(), e.what());
            QMessageBox::warning(this, "Lỗi", "Không thể tải file YAML: " + yaml_file);
        }
    }
}

} // namespace rospital_gui