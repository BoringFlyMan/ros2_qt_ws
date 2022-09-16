#include "robotitem.h"
#include <QPainter>

RobotItem::RobotItem()
{
    m_robotImg.load("://images/robot.png");
    QMatrix matrix;
    matrix.rotate(360);
    m_robotImg = m_robotImg.transformed(matrix, Qt::SmoothTransformation);
}

void RobotItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
//    qDebug()<<"yes ,i remind y";
    drawImage(painter);
    drawPoints(painter);
    drawLines(painter);

    drawMaps(painter);
//    drawrobotPose(painter);
//    drawlocalMaps(painter);
    drawlocalMaps_two(painter);
//    drawglobalMaps(painter);

    drawlaserScan(painter);
}

QRectF RobotItem::boundingRect() const
{
    return QRectF(0, 0, 400, 400);
}

void RobotItem::drawlaserScan(QPainter *painter){
    painter->setPen(QPen(m_lasercolor, 2));
    painter->drawPoints(m_laserscanpoints);
}
void RobotItem::drawrobotPose(QPainter *painter){
//    painter->setPen(QPen(QColor(0, 0, 255), 10));
    painter->save();
    painter->translate(QPointF(m_currpose.x, m_currpose.y));
    painter->drawPoint(0, 0);
    painter->rotate(rad2deg(-m_currpose.theta));
//    painter->drawPixmap(m_robotImg.width()/2, m_robotImg.height()/2, m_robotImg);
    painter->drawPixmap(QPoint(-m_robotImg.width()/2, -m_robotImg.height()/2), m_robotImg);
    painter->restore();

//    painter->drawPoint(m_currpose.x, m_currpose.y);
}

void RobotItem::drawglobalMaps(QPainter *painter){
    painter->drawImage(0, 0, m_globalcostmap);
}

void RobotItem::drawlocalMaps(QPainter *painter){
//    painter->drawImage(0, 0, m_localmap);
    painter->save();
    painter->translate(m_lastlocalcostMap.x, m_lastlocalcostMap.y);
    painter->drawPoint(0, 0);
//    painter->rotate(rad2deg(-m_currpose.theta));
//    painter->drawPixmap(m_robotImg.width()/2, m_robotImg.height()/2, m_robotImg);
    QPixmap m_localmap2 = QPixmap::fromImage(m_localcostmap);
    painter->drawPixmap(QPoint(-m_localcostmap.width()/2, -m_localcostmap.height()/2), m_localmap2);
    painter->restore();
}

void RobotItem::drawlocalMaps_two(QPainter *painter){
    painter->save();
    painter->translate(m_lastlocalcostMap.x, m_lastlocalcostMap.y);
    QPixmap m_localmap2 = QPixmap::fromImage(m_localcostmap);
    painter->drawPixmap(QPoint(0, -m_localcostmap.height()), m_localmap2);
//    painter->drawPixmap(0, 0, m_localmap2);
    painter->restore();

}

void RobotItem::drawMaps(QPainter *painter){
    painter->drawImage(0, 0, m_map);
}

void RobotItem::drawImage(QPainter *painter)
{
    painter->drawImage(0, 0, m_image);
}

void RobotItem::drawPoints(QPainter *painter)
{
    painter->setPen(QPen(QColor(0, 0, 255), 20));
    painter->drawPoints(m_pointf);
}

void RobotItem::drawLines(QPainter *painter)
{
    painter->setPen(QPen(QColor(255, 0, 0), 1));
    painter->drawLine(m_line);
}

void RobotItem::updatelasetScan(QPolygonF points){
    qDebug()<<"receive /scan";
    m_laserscanpoints = points;
    update();
}

void RobotItem::updatelaserColor(QColor color){
    m_lasercolor = color;
    update();
}

void RobotItem::updaterobotPose(robotPose robotpose){
//    qDebug()<<"22222"<<robotpose.x<<robotpose.y<<robotpose.theta;
    m_currpose = robotpose;
    update();
}

void RobotItem::updatelocalMap(QImage img){
//    qDebug()<<"localmap img server";
    m_localcostmap = img;
    m_lastlocalcostMap = m_currpose;
    update();
}

void RobotItem::updateglobalMap_two(QImage img, robotPose pose){
    m_localcostmap = img;
    m_lastlocalcostMap = pose;
    update();
}

void RobotItem::updateglobalMap(QImage img){
//    qDebug()<<"globalmap img server";
    m_globalcostmap = img;
    update();
}

void RobotItem::updateMap(QImage img){
//    qDebug()<<"map img server";
    m_map = img;
    update();
}

void RobotItem::updateImage(QImage img)
{
    m_image = img;
    update();
}
void RobotItem::updateQpointf(QPolygonF pointf)
{
    m_pointf = pointf;
    update();
}
void RobotItem::updateQline(QLine line)
{
    m_line = line;
    update();
}


void RobotItem::wheelEvent(QGraphicsSceneWheelEvent *event){
    double bef_scaleValue = m_scaleValue;
    if(event->delta()>0){
        m_scaleValue *= 1.1;
    }else{
        m_scaleValue *= 0.9;
    }
    setScale(m_scaleValue);

    if(event->delta() > 0){
        moveBy(-event->pos().x()*bef_scaleValue * 0.1, -event->pos().y()*bef_scaleValue * 0.1);
    }else{
        moveBy(event->pos().x()*bef_scaleValue * 0.1, event->pos().y()*bef_scaleValue * 0.1);
    }
    update();
}

void RobotItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
    if(m_ismousePress){
        QPointF point = (event->pos() - m_pressPose) * m_scaleValue;
        moveBy(point.x(),point.y());
    }
}
void RobotItem::mousePressEvent(QGraphicsSceneMouseEvent *event) {
    if(event->button() == Qt::LeftButton){
        m_pressPose = event->pos();
        m_ismousePress = true;
    }
}
void RobotItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {
    if(event->button()==Qt::LeftButton){
        m_pressPose = QPointF();
        m_ismousePress = false;
    }
}
