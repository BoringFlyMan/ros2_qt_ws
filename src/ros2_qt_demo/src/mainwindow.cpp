#include "mainwindow.h"
#include "ui_mainwindow.h"

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
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onRecvData(QString msg){
    ui->label->setText(msg);

}


void MainWindow::on_pushButton_clicked()
{
    commNode->start();
}
