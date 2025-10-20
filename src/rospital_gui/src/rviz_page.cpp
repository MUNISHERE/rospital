#include "rospital_gui/rviz_page.h"
#include <rviz_common/display.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QTimer>

namespace rospital_gui {

RVizPage::RVizPage(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<tf2_ros::Buffer> tf_buffer, QWidget* render_widget, QWidget* parent)
    : QWidget(parent),
      node_(node),
      tf_buffer_(tf_buffer),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
      render_panel_(nullptr),
      manager_(nullptr),
      topic_check_timer_(new QTimer(this)),
      map_topic_available_(false),
      tf_topic_available_(false),
      scan_topic_available_(false),
      grid_display_(nullptr),
      laser_scan_display_(nullptr)
{
    if (!render_widget) {
        RCLCPP_ERROR(node_->get_logger(), "Render widget is null");
        QMessageBox::critical(parent, "Lỗi", "Widget RViz không được khởi tạo");
        throw std::runtime_error("Render widget is null");
    }

    // Đảm bảo render_widget hiển thị
    render_widget->setVisible(true);
    render_widget->show();
    QTimer::singleShot(200, this, [this, render_widget]() {
        setupRViz(render_widget);
        connect(topic_check_timer_, &QTimer::timeout, this, &RVizPage::checkTopicsAndInitialize);
        topic_check_timer_->start(1000); // Kiểm tra mỗi 1 giây
    });
}

RVizPage::~RVizPage() {
    topic_check_timer_->stop();
    if (manager_) {
        manager_->stopUpdate();
        delete manager_;
    }
    if (render_panel_) {
        delete render_panel_;
    }
}

void RVizPage::setupRViz(QWidget* render_widget) {
    if (!render_widget) {
        RCLCPP_ERROR(node_->get_logger(), "Render widget is null in setupRViz");
        throw std::runtime_error("Render widget is null in setupRViz");
    }

    // Thiết lập layout cho render_widget
    QVBoxLayout* layout = new QVBoxLayout(render_widget);
    render_panel_ = new rviz_common::RenderPanel(render_widget);
    if (!render_panel_) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RenderPanel");
        throw std::runtime_error("Failed to create RenderPanel");
    }

    layout->addWidget(render_panel_);
    render_widget->setLayout(layout);
    render_panel_->setMinimumSize(400, 400);
    render_panel_->setVisible(true);
    render_panel_->show();

    // Khởi tạo VisualizationManager
    auto clock = node_->get_clock();
    auto ros_node_abstraction = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("rviz_node");
    auto ros_node_abstraction_weak = std::weak_ptr<rviz_common::ros_integration::RosNodeAbstraction>(ros_node_abstraction);
    manager_ = new rviz_common::VisualizationManager(render_panel_, ros_node_abstraction_weak, nullptr, clock);
    if (!manager_) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create VisualizationManager");
        throw std::runtime_error("Failed to create VisualizationManager");
    }

    // Xóa các display mặc định
    try {
        manager_->removeAllDisplays();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to remove default displays: %s", e.what());
        throw;
    }

    render_panel_->initialize(manager_);
    render_panel_->setMouseTracking(true);
    render_panel_->setFocusPolicy(Qt::StrongFocus);

    // Khởi tạo manager
    try {
        manager_->initialize();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to initialize VisualizationManager: %s", e.what());
        throw;
    }

    // Thiết lập view controller
    auto view_manager = manager_->getViewManager();
    if (view_manager) {
        try {
            view_manager->setCurrentViewControllerType("rviz_default_plugins/Orbit");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to set view controller: %s", e.what());
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get ViewManager");
    }

    // Thêm tools
    auto tool_manager = manager_->getToolManager();
    if (tool_manager) {
        try {
            tool_manager->addTool("rviz_common/Interact");
            tool_manager->addTool("rviz_default_plugins/InitialPose");
            tool_manager->addTool("rviz_default_plugins/GoalPose");
            tool_manager->setCurrentTool(tool_manager->getTool(0));
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to add tools: %s", e.what());
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get ToolManager");
    }
}

void RVizPage::checkTopicsAndInitialize() {
    auto topics = node_->get_topic_names_and_types();
    map_topic_available_ = topics.find("/map") != topics.end();
    tf_topic_available_ = topics.find("/tf") != topics.end() || topics.find("/tf_static") != topics.end();
    scan_topic_available_ = topics.find("/scan") != topics.end();

    if (map_topic_available_ && tf_topic_available_) {
        topic_check_timer_->stop();

        QString frame_id = "map";
        if (manager_) {
            try {
                manager_->setFixedFrame(frame_id);
                manager_->startUpdate();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to set fixed frame or start update: %s", e.what());
                return;
            }
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Manager is null in checkTopicsAndInitialize");
            return;
        }

        // Thêm các display với trạng thái không kích hoạt
        grid_display_ = manager_->createDisplay("rviz_default_plugins/Grid", "Grid Display", false);
        if (grid_display_) {
            try {
                grid_display_->subProp("Line Style")->setValue("Lines");
                grid_display_->subProp("Color")->setValue(QColor(Qt::white));
                grid_display_->subProp("Reference Frame")->setValue(frame_id);
                grid_display_->setEnabled(true);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to configure Grid display: %s", e.what());
            }
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to create Grid display");
        }

        auto tf_display = manager_->createDisplay("rviz_default_plugins/TF", "TF Display", false);
        if (tf_display) {
            try {
                tf_display->setEnabled(true);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to enable TF display: %s", e.what());
            }
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to create TF display");
        }

        auto map_display = manager_->createDisplay("rviz_default_plugins/Map", "Map Display", false);
        if (map_display) {
            try {
                map_display->subProp("Topic")->setValue("/map");
                map_display->setEnabled(true);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to configure Map display: %s", e.what());
            }
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to create Map display");
        }

        auto robot_model_display = manager_->createDisplay("rviz_default_plugins/RobotModel", "RobotModel Display", false);
        if (robot_model_display) {
            try {
                robot_model_display->subProp("Description Topic")->setValue("/robot_description");
                robot_model_display->setEnabled(true);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to configure RobotModel display: %s", e.what());
            }
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to create RobotModel display");
        }

        auto marker_display = manager_->createDisplay("rviz_default_plugins/Marker", "Marker Display", false);
        if (marker_display) {
            try {
                marker_display->subProp("Marker Topic")->setValue("/map_markers");
                marker_display->setEnabled(true);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to configure Marker display: %s", e.what());
            }
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to create Marker display");
        }

        if (scan_topic_available_) {
            laser_scan_display_ = manager_->createDisplay("rviz_default_plugins/LaserScan", "LaserScan Display", false);
            if (laser_scan_display_) {
                try {
                    laser_scan_display_->subProp("Topic")->setValue("/scan");
                    laser_scan_display_->subProp("Size (m)")->setValue(0.1);
                    laser_scan_display_->subProp("Color")->setValue(QColor(Qt::green));
                    laser_scan_display_->setEnabled(true);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(node_->get_logger(), "Failed to configure LaserScan display: %s", e.what());
                }
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Failed to create LaserScan display");
            }
        }
    }
}

} // namespace rospital_gui