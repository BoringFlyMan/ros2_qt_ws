#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "rclcomm.h"
#include "robotitem.h"
#include "robotimg.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    rclcomm *commNode;
public slots:
    void onRecvData(QString);
    void updaterobotImgPose(robotPose pose);

private slots:
    void on_laser_scan_color_clicked();
    void on_pushButton_4_clicked();
    void on_pushButton_3_clicked();
    void on_pushButton_2_clicked();
    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;
    QGraphicsScene *m_qGraphicsScene = nullptr;
    RobotItem *m_robotitem = nullptr;
    robotImg *m_robotimg = nullptr;
};
#endif // MAINWINDOW_H
