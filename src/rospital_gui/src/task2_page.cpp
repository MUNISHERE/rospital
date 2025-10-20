#include "rospital_gui/task2_page.h"
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
#include <rviz_common/tool_manager.hpp>

namespace rospital_gui {

Task2Page::Task2Page(std::shared_ptr<rclcpp::Node> node, 
                     std::shared_ptr<tf2_ros::Buffer> tf_buffer, 
                     Ui::MainWindow *ui, 
                     QWidget *parent)
    : RVizPage(node, tf_buffer, ui->map2Wid, parent),
      node_(node),
      tf_buffer_(tf_buffer),
      ui(ui),
      map_dir_path_(QDir::homePath() + "/rospital/rospital_ws/src/rospital_map/map"),
      current_goal_index_(0),
      task_goals_(),
      selected_goals_()
{
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(node_, "/navigate_through_poses");
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    initial_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

    goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            QString goal_name = QString("Goal_%1").arg(task_goals_.size() + 1);
            QString goal_str = QString("%1 (%2, %3, %4)").arg(goal_name)
                              .arg(msg->pose.position.x, 0, 'f', 2)
                              .arg(msg->pose.position.y, 0, 'f', 2)
                              .arg(0.5, 0, 'f', 2);
            this->ui->task2GoalList->addItem(goal_str);
            task_goals_.push_back({goal_name, *msg, 0.5, nullptr, nullptr, nullptr, nullptr});
            RCLCPP_INFO(node_->get_logger(), "Đã thêm mục tiêu: %s", goal_name.toStdString().c_str());
            QMessageBox::information(this, "Thông báo", "Đã thêm mục tiêu: " + goal_name);
            this->ui->taskStatus2->setText("");
        });

    connect(this->ui->task2PlayBtn, &QPushButton::clicked, this, &Task2Page::on_task2PlayBtn_clicked);
    connect(this->ui->task2GoalSelectBtn, &QPushButton::clicked, this, &Task2Page::on_task2GoalSelectBtn_clicked);
    connect(this->ui->task2GoalBtn, &QPushButton::clicked, this, &Task2Page::on_task2GoalBtn_clicked);
    connect(this->ui->cancel2Btn, &QPushButton::clicked, this, &Task2Page::on_cancel2Btn_clicked);
    connect(this->ui->site2Btn, &QPushButton::clicked, this, &Task2Page::on_site2Btn_clicked);

    // Xóa các tab mặc định trong taskList
    this->ui->taskList->clear();

    this->ui->distanceLastStatus2->setText("0 m");
    this->ui->timeLastStatus2->setText("0 s");
    this->ui->goalLastStatus2->setText("0");
    this->ui->taskStatus2->setText("");

    populateGoalList();
}

Task2Page::~Task2Page() {}

QString Task2Page::getMapNameFromParam()
{
    std::string map_path;
    if (!node_->get_parameter("map", map_path)) {
        RCLCPP_WARN(node_->get_logger(), "Không tìm thấy tham số 'map', sử dụng tên bản đồ mặc định");
        return "default_map";
    }
    QFileInfo file_info(QString::fromStdString(map_path));
    return file_info.baseName();
}

void Task2Page::loadRegionsForTasks()
{
    populateGoalList();
}

void Task2Page::onGoalAdded(const geometry_msgs::msg::PoseStamped &pose)
{
    QString goal_name = QString("Goal_%1").arg(task_goals_.size() + 1);
    QString goal_str = QString("%1 (%2, %3, %4)").arg(goal_name)
                      .arg(pose.pose.position.x, 0, 'f', 2)
                      .arg(pose.pose.position.y, 0, 'f', 2)
                      .arg(0.5, 0, 'f', 2);
    ui->task2GoalList->addItem(goal_str);
    task_goals_.push_back({goal_name, pose, 0.5, nullptr, nullptr, nullptr, nullptr}); 
    RCLCPP_INFO(node_->get_logger(), "Đã thêm mục tiêu: %s", goal_name.toStdString().c_str());
    QMessageBox::information(this, "Thông báo", "Đã thêm mục tiêu: " + goal_name);
    this->ui->taskStatus2->setText("");
}

void Task2Page::populateGoalList()
{
    this->ui->task2GoalList->clear();
    QDir dir(map_dir_path_);
    QString map_name = getMapNameFromParam();
    QString yaml_path = dir.filePath(map_name + ".yaml");

    if (!QFileInfo::exists(yaml_path)) {
        RCLCPP_ERROR(node_->get_logger(), "Không tìm thấy file YAML: %s", yaml_path.toStdString().c_str());
        QMessageBox::warning(this, "Lỗi", "Không tìm thấy file YAML: " + yaml_path);
        return;
    }

    try {
        YAML::Node config = YAML::LoadFile(yaml_path.toStdString());
        for (const auto &region : config["regions"]) {
            QString name = QString::fromStdString(region["name"].as<std::string>());
            double x = region["x"].as<double>();
            double y = region["y"].as<double>();
            double size = region["size"].as<double>();

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = node_->get_clock()->now();
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;

            QString goal_str = QString("%1 (%2, %3, %4)").arg(name).arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(size, 0, 'f', 2);
            this->ui->task2GoalList->addItem(goal_str);
            task_goals_.push_back({name, pose, size, nullptr, nullptr, nullptr, nullptr});
        }
        RCLCPP_INFO(node_->get_logger(), "Đã tải %zu mục tiêu từ YAML", task_goals_.size());
    } catch (const YAML::Exception &e) {
        RCLCPP_ERROR(node_->get_logger(), "Lỗi khi tải file YAML: %s", e.what());
        QMessageBox::critical(this, "Lỗi", "Lỗi khi tải file YAML: " + QString::fromStdString(e.what()));
    }
}

void Task2Page::on_task2GoalSelectBtn_clicked()
{
    int index = this->ui->task2GoalList->currentIndex();
    if (index < 0 || index >= static_cast<int>(task_goals_.size())) {
        QMessageBox::warning(this, "Lỗi", "Vui lòng chọn một mục tiêu từ danh sách.");
        RCLCPP_WARN(node_->get_logger(), "Không có mục tiêu nào được chọn trong task2GoalList");
        return;
    }

    // Kiểm tra xem mục tiêu đã được chọn chưa
    for (const auto &goal : selected_goals_) {
        if (goal.name == task_goals_[index].name) {
            QMessageBox::warning(this, "Lỗi", "Mục tiêu đã được chọn!");
            RCLCPP_WARN(node_->get_logger(), "Mục tiêu %s đã được chọn", task_goals_[index].name.toStdString().c_str());
            return;
        }
    }

    // Tạo tab mới
    QWidget *tab_widget = new QWidget();
    QVBoxLayout *layout = new QVBoxLayout(tab_widget);

    QLabel *name_label = new QLabel(task_goals_[index].name);
    QLabel *distance_label = new QLabel("0 m");
    QLabel *time_label = new QLabel("0 s");
    QPushButton *remove_button = new QPushButton("Xóa đích");
    remove_button->setIcon(QIcon(":/white_icons/icons/white_/delete.svg"));
    remove_button->setStyleSheet(
        "QPushButton {"
        "    background-color: #2596be;"
        "    color: #FFFFFF;"
        "    border-radius: 10px;"
        "    padding: 5px;"
        "}"
        "QPushButton:hover {"
        "    background-color: #1a94ff;"
        "}"
        "QPushButton:pressed {"
        "    background-color: #005a9e;"
        "}"
    );

    layout->addWidget(new QLabel("Tên khu vực:"));
    layout->addWidget(name_label);
    layout->addWidget(new QLabel("Khoảng cách:"));
    layout->addWidget(distance_label);
    layout->addWidget(new QLabel("Thời gian ước tính:"));
    layout->addWidget(time_label);
    layout->addWidget(remove_button);
    layout->addStretch();

    // Thêm tab vào taskList
    int tab_index = this->ui->taskList->addTab(tab_widget, task_goals_[index].name);

    // Lưu các widget vào Goal
    task_goals_[index].name_label = name_label;
    task_goals_[index].distance_label = distance_label;
    task_goals_[index].time_label = time_label;
    task_goals_[index].remove_button = remove_button;

    // Thêm mục tiêu vào selected_goals_
    selected_goals_.push_back(task_goals_[index]);
    RCLCPP_INFO(node_->get_logger(), "Đã chọn mục tiêu: %s", task_goals_[index].name.toStdString().c_str());

    // Kết nối nút Xóa
    connect(remove_button, &QPushButton::clicked, this, [this, tab_index]() {
        on_removeGoal_clicked(tab_index);
    });

    // Cập nhật goalLastStatus2
    this->ui->goalLastStatus2->setText(QString("%1").arg(selected_goals_.size()));
    this->ui->taskStatus2->setText("Đã thêm mục tiêu: " + task_goals_[index].name);
}

void Task2Page::on_removeGoal_clicked(int index)
{
    if (index < 0 || index >= static_cast<int>(selected_goals_.size())) {
        RCLCPP_WARN(node_->get_logger(), "Chỉ số tab không hợp lệ khi xóa: %d", index);
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "Xóa mục tiêu: %s", selected_goals_[index].name.toStdString().c_str());

    // Xóa tab khỏi taskList
    this->ui->taskList->removeTab(index);

    // Xóa mục tiêu khỏi selected_goals_
    selected_goals_.erase(selected_goals_.begin() + index);

    // Cập nhật goalLastStatus2
    this->ui->goalLastStatus2->setText(QString("%1").arg(selected_goals_.size()));
    this->ui->taskStatus2->setText("Đã xóa mục tiêu");

    // Cập nhật lại các kết nối cho nút Xóa
    for (size_t i = 0; i < selected_goals_.size(); ++i) {
        disconnect(selected_goals_[i].remove_button, nullptr, nullptr, nullptr);
        connect(selected_goals_[i].remove_button, &QPushButton::clicked, this, [this, i]() {
            on_removeGoal_clicked(i);
        });
    }
}

void Task2Page::on_task2PlayBtn_clicked()
{
    sendMultiGoals();
}

void Task2Page::sendMultiGoals()
{
    if (selected_goals_.empty()) {
        this->ui->taskStatus2->setText("Lỗi: Chưa chọn mục tiêu nào");
        this->ui->distanceLastStatus2->setText("0 m");
        this->ui->timeLastStatus2->setText("0 s");
        this->ui->goalLastStatus2->setText("0");
        QMessageBox::warning(this, "Lỗi", "Vui lòng chọn ít nhất một mục tiêu từ task2GoalList.");
        RCLCPP_WARN(node_->get_logger(), "Không có mục tiêu nào được chọn để điều hướng");
        return;
    }

    if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
        this->ui->taskStatus2->setText("Lỗi: Không thể kết nối với action server");
        this->ui->distanceLastStatus2->setText("0 m");
        this->ui->timeLastStatus2->setText("0 s");
        this->ui->goalLastStatus2->setText("0");
        QMessageBox::critical(this, "Lỗi", "Không thể kết nối với /navigate_through_poses action server.");
        RCLCPP_ERROR(node_->get_logger(), "Không thể kết nối với /navigate_through_poses action server");
        return;
    }

    auto goal_msg = nav2_msgs::action::NavigateThroughPoses::Goal();
    for (const auto &goal : selected_goals_) {
        goal_msg.poses.push_back(goal.pose);
    }

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.feedback_callback = 
        [this](auto, const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback) {
            double distance = feedback->distance_remaining;
            double time_remaining = feedback->estimated_time_remaining.sec + 
                                   feedback->estimated_time_remaining.nanosec / 1e9;
            int goals_remaining = selected_goals_.size() - current_goal_index_;

            // Cập nhật các tab
            for (size_t i = 0; i < selected_goals_.size(); ++i) {
                if (i == static_cast<size_t>(current_goal_index_)) {
                    selected_goals_[i].distance_label->setText(QString("%1 m").arg(distance, 0, 'f', 2));
                    selected_goals_[i].time_label->setText(QString("%1 s").arg(time_remaining, 0, 'f', 2));
                } else if (i > static_cast<size_t>(current_goal_index_)) {
                    selected_goals_[i].distance_label->setText("0 m");
                    selected_goals_[i].time_label->setText("0 s");
                }
            }

            this->ui->distanceLastStatus2->setText(QString("%1 m").arg(distance, 0, 'f', 2));
            this->ui->timeLastStatus2->setText(QString("%1 s").arg(time_remaining, 0, 'f', 2));
            this->ui->goalLastStatus2->setText(QString("%1").arg(goals_remaining));
            if (current_goal_index_ >= 0 && current_goal_index_ < static_cast<int>(selected_goals_.size())) {
                this->ui->taskStatus2->setText("Đang điều hướng đến mục tiêu: " + selected_goals_[current_goal_index_].name);
                RCLCPP_INFO(node_->get_logger(), "Đang điều hướng đến mục tiêu %d: %s", current_goal_index_, 
                            selected_goals_[current_goal_index_].name.toStdString().c_str());
            } else {
                this->ui->taskStatus2->setText("");
                RCLCPP_WARN(node_->get_logger(), "Chỉ số mục tiêu hiện tại không hợp lệ: %d", current_goal_index_);
            }
        };

    send_goal_options.result_callback = 
        [this](const auto &result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                this->ui->taskStatus2->setText("Hoàn thành nhiệm vụ");
                current_goal_index_ = selected_goals_.size();
                for (auto &goal : selected_goals_) {
                    goal.distance_label->setText("0 m");
                    goal.time_label->setText("0 s");
                }
                this->ui->distanceLastStatus2->setText("0 m");
                this->ui->timeLastStatus2->setText("0 s");
                this->ui->goalLastStatus2->setText("0");
                RCLCPP_INFO(node_->get_logger(), "Nhiệm vụ điều hướng hoàn thành");
            } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
                this->ui->taskStatus2->setText("Nhiệm vụ bị hủy");
                current_goal_index_ = 0;
                RCLCPP_INFO(node_->get_logger(), "Nhiệm vụ điều hướng bị hủy");
            } else {
                this->ui->taskStatus2->setText("Lỗi: Nhiệm vụ thất bại");
                current_goal_index_ = 0;
                RCLCPP_ERROR(node_->get_logger(), "Nhiệm vụ điều hướng thất bại với mã lỗi: %d", static_cast<int>(result.code));
            }
        };

    send_goal_options.goal_response_callback = 
        [this](auto) {
            RCLCPP_INFO(node_->get_logger(), "Đã gửi yêu cầu mục tiêu đến /navigate_through_poses");
        };

    nav_client_->async_send_goal(goal_msg, send_goal_options);
    current_goal_index_ = 0;
    this->ui->taskStatus2->setText("Đã gửi danh sách mục tiêu");
    RCLCPP_INFO(node_->get_logger(), "Đã gửi %zu mục tiêu đến /navigate_through_poses", selected_goals_.size());
}

void Task2Page::on_task2GoalBtn_clicked()
{
    if (manager_) {
        auto tool_manager = manager_->getToolManager();
        if (tool_manager) {
            tool_manager->setCurrentTool(tool_manager->getTool(1)); // GoalPose tool
            RCLCPP_INFO(node_->get_logger(), "Đã kích hoạt công cụ GoalPose");
            QMessageBox::information(this, "Thông báo", "Chọn mục tiêu trên bản đồ");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Tool manager là null");
            QMessageBox::critical(this, "Lỗi", "Không thể truy cập tool manager.");
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Manager là null. Kiểm tra rviz_page.h và rviz_page.cpp.");
        QMessageBox::critical(this, "Lỗi", "Không thể kích hoạt công cụ GoalPose. Vui lòng kiểm tra khởi tạo manager trong RVizPage.");
    }
}

void Task2Page::on_cancel2Btn_clicked()
{
    nav_client_->async_cancel_all_goals();
    this->ui->taskStatus2->setText("Đã hủy nhiệm vụ");
    this->ui->distanceLastStatus2->setText("0 m");
    this->ui->timeLastStatus2->setText("0 s");
    this->ui->goalLastStatus2->setText(QString("%1").arg(selected_goals_.size()));
    for (auto &goal : selected_goals_) {
        goal.distance_label->setText("0 m");
        goal.time_label->setText("0 s");
    }
    current_goal_index_ = 0;
    RCLCPP_INFO(node_->get_logger(), "Đã hủy nhiệm vụ điều hướng");
}

void Task2Page::on_site2Btn_clicked()
{
    if (manager_) {
        auto tool_manager = manager_->getToolManager();
        if (tool_manager) {
            tool_manager->setCurrentTool(tool_manager->getTool(0)); // InitialPose tool
            RCLCPP_INFO(node_->get_logger(), "Đã kích hoạt công cụ InitialPose");
            QMessageBox::information(this, "Thông báo", "Chọn vị trí bắt đầu trên bản đồ");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Tool manager là null");
            QMessageBox::critical(this, "Lỗi", "Không thể truy cập tool manager.");
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Manager là null. Kiểm tra rviz_page.h và rviz_page.cpp.");
        QMessageBox::critical(this, "Lỗi", "Không thể kích hoạt công cụ InitialPose. Vui lòng kiểm tra khởi tạo manager trong RVizPage.");
    }
}

} // namespace rospital_gui
