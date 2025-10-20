#ifndef RVIZ_PAGE_H
#define RVIZ_PAGE_H

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/render_panel.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <QWidget>
#include <QTimer>

namespace rospital_gui {

class RVizPage : public QWidget {
    Q_OBJECT
public:
    RVizPage(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<tf2_ros::Buffer> tf_buffer, QWidget* render_widget, QWidget* parent = nullptr);
    virtual ~RVizPage();

protected:
    void setupRViz(QWidget* render_widget);
    void checkTopicsAndInitialize();

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rviz_common::RenderPanel* render_panel_;
    rviz_common::VisualizationManager* manager_;
    QTimer* topic_check_timer_;
    bool map_topic_available_;
    bool tf_topic_available_;
    bool scan_topic_available_;
    rviz_common::Display* grid_display_;
    rviz_common::Display* laser_scan_display_;
};

} // namespace rospital_gui

#endif // RVIZ_PAGE_H
