#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QGraphicsScene>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    commNode = new rclcomm();
    //start use run function;
    //commNode->start();
    connect(commNode, SIGNAL(emitTopicData(QString)), this, SLOT(onRecvData(QString)));
    QImage img;
    img.load("://images/test.png");
    ui->label_3->setPixmap(QPixmap::fromImage(img).scaled(ui->label_3->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));//

    m_qGraphicsScene = new QGraphicsScene();
    m_qGraphicsScene->clear();

    m_robotitem = new RobotItem();
    m_robotimg = new robotImg();

    m_robotitem->setZValue(1);
    m_robotimg->setZValue(10);

    m_qGraphicsScene->addItem(m_robotitem);
    m_qGraphicsScene->addItem(m_robotimg);

    ui->graphicsView->setScene(m_qGraphicsScene);
    connect(commNode, SIGNAL(emitupdateMap(QImage)), m_robotitem, SLOT(updateMap(QImage)));
//    connect(commNode, SIGNAL(emitupdatelocalMap(QImage)), m_robotitem, SLOT(updatelocalMap(QImage)));
    connect(commNode, SIGNAL(emitupdatelocalMap_two(QImage, robotPose)),m_robotitem, SLOT(updateglobalMap_two(QImage, robotPose)));
    connect(commNode, SIGNAL(emitupdateglobalMap(QImage)), m_robotitem, SLOT(updateglobalMap(QImage)));
    connect(commNode, SIGNAL(emitupdaterobotPose(robotPose)), m_robotitem, SLOT(updaterobotPose(robotPose)));
    connect(commNode, SIGNAL(emitupdatelaserPoints(QPolygonF)), m_robotitem, SLOT(updatelasetScan(QPolygonF)));
    connect(commNode, SIGNAL(emituodateglobalPath(QPolygonF)), m_robotitem, SLOT(updateglobalPath(QPolygonF)));
    connect(commNode, SIGNAL(emituodatelocalPath(QPolygonF)), m_robotitem, SLOT(updatelocalPath(QPolygonF)));

    //m_robotimg这里对于机器人位姿的处理先转换到mianwindow下，然后图片与整个robotimg坐标同步移动，让robotimg下的图片获取旋转角度单独转动。
    connect(commNode, SIGNAL(emitupdaterobotPose(robotPose)), this, SLOT(updaterobotImgPose(robotPose)));

    commNode->start();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::updaterobotImgPose(robotPose pose){
    m_robotitem->updaterobotPose(pose);
    QPointF robotpose;
    robotpose.setX(pose.x);
    robotpose.setY(pose.y);
    QPointF scenePose = m_robotitem->mapToScene(robotpose);
    m_robotimg->updaterobotPose(pose);
    m_robotimg->setPos(scenePose);
}

void MainWindow::onRecvData(QString msg){
    ui->label->setText(msg);
}

void MainWindow::on_pushButton_clicked()
{

}

void MainWindow::on_pushButton_2_clicked()
{
    QImage img;
    img.load("://images/test.png");
    m_robotitem->updateImage(img);
}

void MainWindow::on_pushButton_3_clicked()
{
    QPolygonF point;
    point.push_back(QPointF(0,0));
    m_robotitem->updateQpointf(point);
}

void MainWindow::on_pushButton_4_clicked()
{
    QLine line(QPoint(0,0),QPoint(30,30));
    m_robotitem->updateQline(line);
}

void MainWindow::on_laser_scan_color_clicked()
{
    int R = ui->lineEdit->text().toInt();
    int G = ui->lineEdit_2->text().toUInt();
    int B = ui->lineEdit_3->text().toUInt();
    QColor color;
    color.setRgb(R, G, B);
    m_robotitem->updatelaserColor(color);
}




