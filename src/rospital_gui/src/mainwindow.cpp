#include "rospital_gui/mainwindow.h"
#include <QApplication>
#include <QFileInfo>
#include <QMessageBox>

namespace rospital_gui {

MainWindow::MainWindow(std::shared_ptr<rclcpp::Node> node, QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), node_(node)
{
    ui->setupUi(this);
    if (!ui) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to initialize UI");
        QMessageBox::critical(nullptr, "Lỗi", "Không thể khởi tạo giao diện người dùng");
        throw std::runtime_error("UI initialization failed");
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    map_page_ = new MapPage(node_, tf_buffer_, ui, this);
    task_page_ = new TaskPage(ui, this);
    task1_page_ = new Task1Page(node_, tf_buffer_, ui, this);
    task2_page_ = new Task2Page(node_, tf_buffer_, ui, this);

    // Thêm các trang vào stackedWidget
    ui->stackedWidget->addWidget(map_page_);
    ui->stackedWidget->addWidget(task_page_);
    ui->stackedWidget->addWidget(task1_page_);
    ui->stackedWidget->addWidget(task2_page_);
    ui->stackedWidget->setCurrentWidget(map_page_);

    // Kết nối các nút menu
    connect(ui->mapBtn, &QPushButton::clicked, this, &MainWindow::on_mapBtn_clicked);
    connect(ui->taskBtn, &QPushButton::clicked, this, &MainWindow::on_taskBtn_clicked);
    connect(ui->helpBtn, &QPushButton::clicked, this, &MainWindow::on_helpBtn_clicked);
    connect(ui->aboutBtn, &QPushButton::clicked, this, &MainWindow::on_aboutBtn_clicked);
    connect(ui->settingBtn, &QPushButton::clicked, this, &MainWindow::on_settingBtn_clicked);

    // Kết nối tín hiệu từ TaskPage
    connect(task_page_, &TaskPage::task1Selected, this, [this]() { ui->stackedWidget->setCurrentWidget(task1_page_); });
    connect(task_page_, &TaskPage::task2Selected, this, [this]() { ui->stackedWidget->setCurrentWidget(task2_page_); });

    // Kết nối tín hiệu từ MapPage
    connect(map_page_, &MapPage::goalAdded, task1_page_, [this](const QString&, const QString&, const geometry_msgs::msg::PoseStamped& pose, double) {
        task1_page_->onGoalAdded(pose);
    });
    connect(map_page_, &MapPage::goalAdded, task2_page_, [this](const QString&, const QString&, const geometry_msgs::msg::PoseStamped& pose, double) {
        task2_page_->onGoalAdded(pose);
    });
    connect(map_page_, &MapPage::mapLoaded, task1_page_, &Task1Page::loadRegionsForTasks);
    connect(map_page_, &MapPage::mapLoaded, task2_page_, &Task2Page::loadRegionsForTasks);

    // Cập nhật trạng thái ban đầu
    std::string map_path;
    if (node_->get_parameter("map", map_path)) {
        QFileInfo file_info(QString::fromStdString(map_path));
        QString map_name = file_info.baseName();
        ui->mapStatus->setText("Khởi tạo bản đồ: " + map_name);
        ui->distanceStatus1->setText("Khởi tạo bản đồ: " + map_name);
        ui->taskStatus2->setText("Khởi tạo bản đồ: " + map_name);
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Map parameter not found");
        ui->mapStatus->setText("Lỗi: Không tìm thấy tham số map");
        ui->distanceStatus1->setText("Lỗi: Không tìm thấy tham số map");
        ui->taskStatus2->setText("Lỗi: Không tìm thấy tham số map");
    }
}

MainWindow::~MainWindow()
{
    delete map_page_;
    delete task_page_;
    delete task1_page_;
    delete task2_page_;
    delete ui;
}

void MainWindow::on_taskBtn_clicked()
{
    ui->stackedWidget->setCurrentWidget(task_page_);
}

void MainWindow::on_mapBtn_clicked()
{
    ui->stackedWidget->setCurrentWidget(map_page_);
}

void MainWindow::on_helpBtn_clicked()
{
    ui->stackedWidget->setCurrentWidget(ui->helpPg);
}

void MainWindow::on_aboutBtn_clicked()
{
    ui->stackedWidget->setCurrentWidget(ui->aboutPg);
}

void MainWindow::on_settingBtn_clicked()
{
    ui->stackedWidget->setCurrentWidget(ui->settingPg);
}

} // namespace rospital_gui