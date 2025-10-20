#ifndef TASK_PAGE_H
#define TASK_PAGE_H

#include <QWidget>
#include "ui_mainwindow.h"

namespace rospital_gui {

class TaskPage : public QWidget
{
    Q_OBJECT

public:
    explicit TaskPage(Ui::MainWindow *ui, QWidget *parent = nullptr);
    ~TaskPage();

signals:
    void task1Selected();
    void task2Selected();
    void task3Selected();
    void task4Selected();

private slots:
    void on_task1Btn_clicked();
    void on_task2Btn_clicked();
    void on_task3Btn_clicked();
    void on_task4Btn_clicked();

private:
    Ui::MainWindow *ui;
};

} // namespace rospital_gui

#endif // ROSPITAL_GUI_TASK_PAGE_H