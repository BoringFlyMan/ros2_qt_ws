#ifndef ROBOTIMG_H
#define ROBOTIMG_H

#include <QObject>
#include <QGraphicsItem>
#include "RobotAlgorithm.h"

class robotImg:public QObject, public QGraphicsItem
{
    Q_OBJECT
public:
    robotImg();
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = nullptr) override;

    QRectF boundingRect() const override;
private:
    QPixmap m_robotImg;
    robotPose m_currRobotPose;

private:
    void drawrobotImg(QPainter *painter);

public slots:
    void updaterobotPose(robotPose robotpose);

};

#endif // ROBOTIMG_H
