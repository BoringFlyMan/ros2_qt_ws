#ifndef ROBOTITEM_H
#define ROBOTITEM_H

#include <QObject>
#include <QDebug>
#include <QGraphicsItem>
#include <QGraphicsSceneWheelEvent>
#include "RobotAlgorithm.h"

class RobotItem :public QObject, public QGraphicsItem
{
    Q_OBJECT
public:
    RobotItem();
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = nullptr) override;

    QRectF boundingRect() const override;

    void wheelEvent(QGraphicsSceneWheelEvent *event) override;
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
    void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;

    void updateImage(QImage img);
    void updateQpointf(QPolygonF pointf);
    void updateQline(QLine line);


public slots:
    void updateMap(QImage img);
    void updatelocalMap(QImage img);
    void updateglobalMap_two(QImage img, robotPose pose);
    void updateglobalMap(QImage img);
    void updaterobotPose(robotPose robotpose);
    void updatelasetScan(QPolygonF points);
    void updatelaserColor(QColor color);


private:
    QImage m_image;
    QPolygonF m_pointf;
    QLine m_line;

    QImage m_map;
    QImage m_localcostmap;
    robotPose m_lastlocalcostMap;
    QColor m_lasercolor = QColor(255, 0, 0);

    QImage m_globalcostmap;
    robotPose m_currpose;
    QPixmap m_robotImg;
    QPolygonF m_laserscanpoints;

    double m_scaleValue = 1;

    bool m_ismousePress;
    QPointF m_pressPose;

    void drawImage(QPainter *painter);
    void drawPoints(QPainter *painter);
    void drawLines(QPainter *painter);
    void drawMaps(QPainter *painter);
    void drawlocalMaps(QPainter *painter);
    void drawlocalMaps_two(QPainter *painter);
    void drawglobalMaps(QPainter *painter);
    void drawrobotPose(QPainter *painter);
    void drawlaserScan(QPainter *painter);
};

#endif // ROBOTITEM_H
