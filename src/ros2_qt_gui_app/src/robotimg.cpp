#include "robotimg.h"
#include <QPainter>

robotImg::robotImg()
{
    m_robotImg.load("://images/robot.png");
    QMatrix matrix;
    matrix.rotate(360);
    m_robotImg = m_robotImg.transformed(matrix, Qt::SmoothTransformation);
}

void robotImg::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    drawrobotImg(painter);

}

QRectF robotImg::boundingRect() const
{
    return QRectF(0, 0, m_robotImg.width(), m_robotImg.height());

}

void robotImg::drawrobotImg(QPainter *painter){
    painter->setRenderHint(QPainter::Antialiasing, true);//设置反锯齿
    painter->save();
    painter->rotate(rad2deg(-m_currRobotPose.theta));
    painter->drawPixmap(QPoint(-m_robotImg.width()/2, -m_robotImg.height()/2), m_robotImg);
    painter->restore();
}

void robotImg::updaterobotPose(robotPose robotpose){
    m_currRobotPose = robotpose;
    update();
}


