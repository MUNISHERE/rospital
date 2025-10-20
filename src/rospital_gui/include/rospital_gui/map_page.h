#ifndef MAP_PAGE_H
#define MAP_PAGE_H

#include "rospital_gui/rviz_page.h" 
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/render_panel.hpp>
#include <QInputDialog>
#include <yaml-cpp/yaml.h>
#include <random>

#include <QDir>
#include "ui_mainwindow.h"

namespace rospital_gui {

class MapPage : public RVizPage // Kế thừa từ RVizPage thay vì QWidget
{
    Q_OBJECT
public:
    explicit MapPage(std::shared_ptr<rclcpp::Node> node, 
                     std::shared_ptr<tf2_ros::Buffer> tf_buffer, // Thêm tf_buffer
                     Ui::MainWindow *ui, 
                     QWidget *parent = nullptr);
    ~MapPage();

    struct Region {
        QString name;
        QString shape_type;
        geometry_msgs::msg::PoseStamped pose;
        double size;
    };

    enum class Color { RED, GREEN, BLUE };

    struct HistoryAction {
        enum class Type { ADD, REMOVE };
        Type type;
        Region region;
    };

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    bool eventFilter(QObject *obj, QEvent *event) override;

private slots:
    void on_saveChangeBtn_clicked();
    void on_clearBtn_clicked();
    void on_circleBtn_clicked();
    void on_squareBtn_clicked();
    void on_pointBtn_clicked();
    void on_selectRegionBtn_clicked();
    void on_undoBtn_clicked();
    void on_redoBtn_clicked();

private:
    void loadMap();
    void loadRegions(const QString &map_name);
    void saveRegions(const QString &map_name);
    void publishMarkers();
    void addRegion(const QString &name, const QString &shape_type, const geometry_msgs::msg::PoseStamped &pose, double size);
    void clearRegions();
    void handleGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void handleMarkerSelect(const visualization_msgs::msg::Marker::SharedPtr msg);
    bool isValidRegionName(const QString &name);
    QString getMapNameFromParam();

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr map_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_select_sub_;
    Ui::MainWindow *ui;
    rviz_common::RenderPanel *render_panel_;
    rviz_common::VisualizationManager *manager_;
    QString map_dir_path_;
    QString current_shape_type_;
    double current_size_;
    Color current_color_;
    std::vector<Region> regions_;
    std::vector<HistoryAction> history_;
    size_t history_index_;
    int current_tool_index_;
    std::mt19937 rng_;
    bool is_drawing_;
    QPointF start_point_;

signals:
    void goalAdded(const QString &name, const QString &shape_type, const geometry_msgs::msg::PoseStamped &pose, double size);
    void mapLoaded(const QString &map_name);
};

} // namespace rospital_gui

#endif // MAP_PAGE_H