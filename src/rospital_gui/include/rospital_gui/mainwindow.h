#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "ui_mainwindow.h"
#include "rospital_gui/map_page.h"
#include "rospital_gui/task_page.h"
#include "rospital_gui/task1_page.h"
#include "rospital_gui/task2_page.h"

namespace rospital_gui {

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(std::shared_ptr<rclcpp::Node> node, QWidget *parent = nullptr); // Constructor mới
    ~MainWindow();

private slots:
    void on_taskBtn_clicked();
    void on_mapBtn_clicked();
    void on_helpBtn_clicked();
    void on_aboutBtn_clicked();
    void on_settingBtn_clicked();

private:
    Ui::MainWindow *ui;
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    MapPage *map_page_;
    TaskPage *task_page_;
    Task1Page *task1_page_;
    Task2Page *task2_page_;
};

} // namespace rospital_gui

#endif // MAINWINDOW_H