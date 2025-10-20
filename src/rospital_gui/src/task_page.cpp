#include "rospital_gui/task_page.h"

namespace rospital_gui {

TaskPage::TaskPage(Ui::MainWindow *ui, QWidget *parent)
    : QWidget(parent), ui(ui)
{
    connect(ui->task1Btn, &QPushButton::clicked, this, &TaskPage::on_task1Btn_clicked);
    connect(ui->task2Btn, &QPushButton::clicked, this, &TaskPage::on_task2Btn_clicked);
    connect(ui->task3Btn, &QPushButton::clicked, this, &TaskPage::on_task3Btn_clicked);
    connect(ui->task4Btn, &QPushButton::clicked, this, &TaskPage::on_task4Btn_clicked);
}

TaskPage::~TaskPage()
{
}

void TaskPage::on_task1Btn_clicked()
{
    emit task1Selected();
}

void TaskPage::on_task2Btn_clicked()
{
    emit task2Selected();
}

void TaskPage::on_task3Btn_clicked()
{
    emit task3Selected();
}

void TaskPage::on_task4Btn_clicked()
{
    emit task4Selected();
}

} // namespace rospital_gui