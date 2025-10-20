#include "rospital_gui/map_page.h"
#include <yaml-cpp/yaml.h>
#include <random>
#include <rviz_default_plugins/displays/map/map_display.hpp>
#include <rviz_default_plugins/displays/marker/marker_display.hpp>
#include <rviz_default_plugins/tools/pose_estimate/initial_pose_tool.hpp>
#include <rviz_default_plugins/tools/goal_pose/goal_tool.hpp>

#include <QVBoxLayout>
#include <QMessageBox>
#include <QMouseEvent>

namespace rospital_gui {

MapPage::MapPage(std::shared_ptr<rclcpp::Node> node, 
                 std::shared_ptr<tf2_ros::Buffer> tf_buffer, 
                 Ui::MainWindow *ui, 
                 QWidget *parent)
    : RVizPage(node, tf_buffer, ui->map5Wid, parent), 
      node_(node),
      tf_buffer_(tf_buffer),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
      marker_pub_(node->create_publisher<visualization_msgs::msg::MarkerArray>("/map_markers", 10)),
      map_client_(node->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map")),
      goal_sub_(node->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/goal_pose", 10, std::bind(&MapPage::handleGoalPose, this, std::placeholders::_1))),
      marker_select_sub_(node->create_subscription<visualization_msgs::msg::Marker>(
          "/rviz_selected_marker", 10, std::bind(&MapPage::handleMarkerSelect, this, std::placeholders::_1))),
      ui(ui),
      map_dir_path_("/home/rospital/rospital_ws/install/rospital_map/share/rospital_map/map"),
      current_shape_type_("circle"),
      current_size_(0.5),
      current_color_(Color::GREEN),
      regions_(),
      history_(),
      history_index_(0),
      current_tool_index_(2),
      rng_(std::random_device()()),
      is_drawing_(false),
      start_point_()
{
    loadMap(); // Tải bản đồ từ tham số ROS 2

    connect(ui->saveChangeBtn, &QPushButton::clicked, this, &MapPage::on_saveChangeBtn_clicked);
    connect(ui->clearChangeBtn, &QPushButton::clicked, this, &MapPage::on_clearBtn_clicked);
    connect(ui->circleBtn, &QPushButton::clicked, this, &MapPage::on_circleBtn_clicked);
    connect(ui->squareBtn, &QPushButton::clicked, this, &MapPage::on_squareBtn_clicked);
    connect(ui->pointBtn, &QPushButton::clicked, this, &MapPage::on_pointBtn_clicked);
    connect(ui->selectRegionBtn, &QPushButton::clicked, this, &MapPage::on_selectRegionBtn_clicked);
    connect(ui->undoBtn, &QPushButton::clicked, this, &MapPage::on_undoBtn_clicked);
    connect(ui->redoBtn, &QPushButton::clicked, this, &MapPage::on_redoBtn_clicked);

    render_panel_->installEventFilter(this);
}

MapPage::~MapPage() {}

QString MapPage::getMapNameFromParam()
{
    std::string map_path;
    // Thử lấy tham số trong namespace rỗng
    if (node_->get_parameter("map", map_path)) {
        RCLCPP_INFO(node_->get_logger(), "Found map parameter: %s", map_path.c_str());
    } else {
        // Thử lấy tham số trong namespace của node
        std::string namespaced_param = node_->get_namespace();
        if (namespaced_param == "/") namespaced_param = "";
        namespaced_param += "/map";
        if (node_->get_parameter(namespaced_param, map_path)) {
            RCLCPP_INFO(node_->get_logger(), "Found map parameter in namespace: %s", map_path.c_str());
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get map parameter in global or namespaced scope");
            ui->mapStatus->setText("Lỗi: Không tìm thấy tham số map");
            return QString("small_house"); // Fallback to default map
        }
    }
    QFileInfo file_info(QString::fromStdString(map_path));
    return file_info.baseName();
}

void MapPage::loadMap()
{
    QString map_name = getMapNameFromParam();
    auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    request->map_url = (map_dir_path_ + "/" + map_name + ".yaml").toStdString();

    if (!map_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node_->get_logger(), "Map server not available");
        ui->mapStatus->setText("Lỗi: Không kết nối được với map server");
        return;
    }

    auto result = map_client_->async_send_request(request);
    result.wait_for(std::chrono::seconds(5));
    if (result.valid() && result.get()->result == nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
        loadRegions(map_name);
        emit mapLoaded(map_name);
        RCLCPP_INFO(node_->get_logger(), "Loaded map: %s", map_name.toStdString().c_str());
        ui->mapStatus->setText("Đã tải bản đồ: " + map_name);
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load map: %s", map_name.toStdString().c_str());
        ui->mapStatus->setText("Lỗi tải bản đồ: " + map_name);
    }
}

void MapPage::loadRegions(const QString &map_name)
{
    QString yaml_file = map_dir_path_ + "/" + map_name + ".yaml";
    try {
        YAML::Node yaml = YAML::LoadFile(yaml_file.toStdString());
        regions_.clear();
        history_.clear();
        history_index_ = 0;
        for (const auto &region : yaml["regions"]) {
            Region reg;
            reg.name = QString::fromStdString(region["name"].as<std::string>());
            reg.shape_type = QString::fromStdString(region["shape_type"].as<std::string>());
            reg.pose.header.frame_id = "map";
            reg.pose.header.stamp = node_->now();
            reg.pose.pose.position.x = region["x"].as<double>();
            reg.pose.pose.position.y = region["y"].as<double>();
            reg.pose.pose.position.z = region["z"] ? region["z"].as<double>() : 0.0;
            reg.pose.pose.orientation.x = region["orientation_x"] ? region["orientation_x"].as<double>() : 0.0;
            reg.pose.pose.orientation.y = region["orientation_y"] ? region["orientation_y"].as<double>() : 0.0;
            reg.pose.pose.orientation.z = region["orientation_z"] ? region["orientation_z"].as<double>() : 0.0;
            reg.pose.pose.orientation.w = region["orientation_w"] ? region["orientation_w"].as<double>() : 1.0;
            reg.size = region["size"].as<double>();
            regions_.push_back(reg);
        }
        publishMarkers();
        ui->mapStatus->setText("Đã tải vùng từ: " + yaml_file);
    } catch (const YAML::Exception &e) {
        RCLCPP_WARN(node_->get_logger(), "Failed to load regions from %s: %s", yaml_file.toStdString().c_str(), e.what());
        ui->mapStatus->setText("Lỗi tải vùng từ: " + yaml_file);
    }
}

void MapPage::saveRegions(const QString &map_name)
{
    QString yaml_file = map_dir_path_ + "/" + map_name + ".yaml";
    YAML::Node yaml;
    try {
        yaml = YAML::LoadFile(yaml_file.toStdString());
    } catch (const YAML::Exception &e) {
        RCLCPP_WARN(node_->get_logger(), "Creating new YAML file: %s", yaml_file.toStdString().c_str());
    }

    YAML::Node regions_node;
    for (const auto &region : regions_) {
        YAML::Node region_node;
        region_node["name"] = region.name.toStdString();
        region_node["shape_type"] = region.shape_type.toStdString();
        region_node["x"] = region.pose.pose.position.x;
        region_node["y"] = region.pose.pose.position.y;
        region_node["z"] = region.pose.pose.position.z;
        region_node["orientation_x"] = region.pose.pose.orientation.x;
        region_node["orientation_y"] = region.pose.pose.orientation.y;
        region_node["orientation_z"] = region.pose.pose.orientation.z;
        region_node["orientation_w"] = region.pose.pose.orientation.w;
        region_node["size"] = region.size;
        regions_node.push_back(region_node);
    }
    yaml["regions"] = regions_node;

    std::ofstream fout(yaml_file.toStdString());
    fout << yaml;
    fout.close();
    RCLCPP_INFO(node_->get_logger(), "Saved regions to %s", yaml_file.toStdString().c_str());
    ui->mapStatus->setText("Đã lưu vùng vào: " + yaml_file);
}

void MapPage::publishMarkers()
{
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    for (const auto &region : regions_) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = node_->now();
        marker.ns = "regions";
        marker.id = id++;
        marker.type = region.shape_type == "circle" ? visualization_msgs::msg::Marker::SPHERE :
                      region.shape_type == "square" ? visualization_msgs::msg::Marker::CUBE :
                      visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = region.pose.pose;
        marker.scale.x = region.shape_type == "point" ? 0.2 : region.size;
        marker.scale.y = region.shape_type == "point" ? 0.2 : region.size;
        marker.scale.z = region.shape_type == "point" ? 0.2 : region.size;
        marker.color.r = current_color_ == Color::RED ? 1.0 : 0.0;
        marker.color.g = current_color_ == Color::GREEN ? 1.0 : 0.0;
        marker.color.b = current_color_ == Color::BLUE ? 1.0 : 0.0;
        marker.color.a = 0.6;
        marker.text = region.name.toStdString();
        marker_array.markers.push_back(marker);
    }
    marker_pub_->publish(marker_array);
}

void MapPage::addRegion(const QString &name, const QString &shape_type, const geometry_msgs::msg::PoseStamped &pose, double size)
{
    Region region;
    region.name = name;
    region.shape_type = shape_type;
    region.pose = pose;
    region.size = size;
    regions_.push_back(region);

    HistoryAction action;
    action.type = HistoryAction::Type::ADD;
    action.region = region;
    if (history_index_ < history_.size()) {
        history_.resize(history_index_);
    }
    history_.push_back(action);
    history_index_++;
}

void MapPage::clearRegions()
{
    for (const auto &region : regions_) {
        HistoryAction action;
        action.type = HistoryAction::Type::REMOVE;
        action.region = region;
        if (history_index_ < history_.size()) {
            history_.resize(history_index_);
        }
        history_.push_back(action);
        history_index_++;
    }
    regions_.clear();
    publishMarkers();
    ui->mapStatus->setText("Đã xóa tất cả vùng");
}

void MapPage::handleGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    QString name = QString("Goal_%1").arg(regions_.size() + 1);
    addRegion(name, current_shape_type_, *msg, current_size_);
    emit goalAdded(name, current_shape_type_, *msg, current_size_);
    publishMarkers();
    ui->mapStatus->setText("Đã thêm vùng: " + name);
}

void MapPage::handleMarkerSelect(const visualization_msgs::msg::Marker::SharedPtr msg)
{
    if (msg->action == visualization_msgs::msg::Marker::DELETE) {
        auto it = std::find_if(regions_.begin(), regions_.end(), [&](const Region &region) {
            return region.name.toStdString() == msg->text;
        });
        if (it != regions_.end()) {
            HistoryAction action;
            action.type = HistoryAction::Type::REMOVE;
            action.region = *it;
            if (history_index_ < history_.size()) {
                history_.resize(history_index_);
            }
            history_.push_back(action);
            history_index_++;
            regions_.erase(it);
            publishMarkers();
            ui->mapStatus->setText("Đã xóa vùng: " + QString::fromStdString(msg->text));
        }
    }
}

bool MapPage::isValidRegionName(const QString &name)
{
    return std::none_of(regions_.begin(), regions_.end(), [&](const Region &region) {
        return region.name == name;
    });
}

void MapPage::on_saveChangeBtn_clicked()
{
    saveRegions(getMapNameFromParam());
}

void MapPage::on_clearBtn_clicked()
{
    clearRegions();
}

void MapPage::on_circleBtn_clicked()
{
    current_shape_type_ = "circle";
    current_tool_index_ = 2;
    is_drawing_ = true;
    ui->mapStatus->setText("Đã chọn công cụ: Vùng tròn");
}

void MapPage::on_squareBtn_clicked()
{
    current_shape_type_ = "square";
    current_tool_index_ = 2;
    is_drawing_ = true;
    ui->mapStatus->setText("Đã chọn công cụ: Vùng vuông");
}

void MapPage::on_pointBtn_clicked()
{
    current_shape_type_ = "point";
    current_tool_index_ = 2;
    is_drawing_ = true;
    ui->mapStatus->setText("Đã chọn công cụ: Điểm");
}

void MapPage::on_selectRegionBtn_clicked()
{
    current_tool_index_ = 3;
    is_drawing_ = false;
    ui->mapStatus->setText("Đã chọn công cụ: Chọn vùng");
}

void MapPage::on_undoBtn_clicked()
{
    if (history_index_ > 0) {
        history_index_--;
        auto &action = history_[history_index_];
        if (action.type == HistoryAction::Type::ADD) {
            regions_.erase(std::remove_if(regions_.begin(), regions_.end(), [&](const Region &region) {
                return region.name == action.region.name;
            }), regions_.end());
            ui->mapStatus->setText("Hoàn tác: Xóa vùng " + action.region.name);
        } else if (action.type == HistoryAction::Type::REMOVE) {
            regions_.push_back(action.region);
            ui->mapStatus->setText("Hoàn tác: Khôi phục vùng " + action.region.name);
        }
        publishMarkers();
    }
}

void MapPage::on_redoBtn_clicked()
{
    if (history_index_ < history_.size()) {
        auto &action = history_[history_index_];
        if (action.type == HistoryAction::Type::ADD) {
            regions_.push_back(action.region);
            ui->mapStatus->setText("Làm lại: Thêm vùng " + action.region.name);
        } else if (action.type == HistoryAction::Type::REMOVE) {
            regions_.erase(std::remove_if(regions_.begin(), regions_.end(), [&](const Region &region) {
                return region.name == action.region.name;
            }), regions_.end());
            ui->mapStatus->setText("Làm lại: Xóa vùng " + action.region.name);
        }
        history_index_++;
        publishMarkers();
    }
}

void MapPage::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && current_tool_index_ == 2) {
        start_point_ = event->pos();
        is_drawing_ = true;
    }
}

void MapPage::mouseMoveEvent(QMouseEvent *event)
{
    if (is_drawing_) {
        QPointF current_point = event->pos();
        double distance = std::sqrt(std::pow(current_point.x() - start_point_.x(), 2) + std::pow(current_point.y() - start_point_.y(), 2)) / 100.0;
        current_size_ = distance;
    }
}

void MapPage::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && is_drawing_) {
        is_drawing_ = false;
        if (!current_shape_type_.isEmpty()) {
            QString name = QInputDialog::getText(this, "Thêm vùng mới", "Tên vùng:");
            if (!name.isEmpty() && isValidRegionName(name)) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.header.stamp = node_->now();
                pose.pose.position.x = start_point_.x() / 100.0;
                pose.pose.position.y = start_point_.y() / 100.0;
                pose.pose.orientation.w = 1.0;
                addRegion(name, current_shape_type_, pose, current_size_);
                emit goalAdded(name, current_shape_type_, pose, current_size_);
                publishMarkers();
                RCLCPP_INFO(node_->get_logger(), "Đã thêm vùng: %s", name.toStdString().c_str());
                ui->mapStatus->setText("Đã thêm vùng: " + name);
            }
        }
        current_size_ = 0.5;
    }
}

bool MapPage::eventFilter(QObject *obj, QEvent *event)
{
    if (obj == render_panel_) {
        if (event->type() == QEvent::MouseButtonPress) {
            mousePressEvent(static_cast<QMouseEvent *>(event));
            return true;
        } else if (event->type() == QEvent::MouseMove) {
            mouseMoveEvent(static_cast<QMouseEvent *>(event));
            return true;
        } else if (event->type() == QEvent::MouseButtonRelease) {
            mouseReleaseEvent(static_cast<QMouseEvent *>(event));
            return true;
        }
    }
    return QWidget::eventFilter(obj, event);
}

} // namespace rospital_gui