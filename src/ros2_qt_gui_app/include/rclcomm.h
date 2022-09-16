#ifndef RCLCOMM_H
#define RCLCOMM_H

#include <QObject>
#include <QThread>

#include <QDebug>
#include <QImage>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>


#include <tf2/LinearMath/QuadWord.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "RobotAlgorithm.h"

class rclcomm :public QThread
{
    Q_OBJECT
public:
    rclcomm();
    void run() override;
    QPointF transWorldPoint2Scene(QPointF point);
    QPointF transScene2WorldPoint(QPointF point);

    void getRobotPose();
    robotPose m_currpose;

private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _publisher;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr _subscription;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _map_sub;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _map_local_sub;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _map_global_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _laser_sub;

    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_transform_listener;
//    rclcpp::executors::SingleThreadedExecutor::SharedPtr m_executor;
    rclcpp::executors::MultiThreadedExecutor *m_executor;
    rclcpp::CallbackGroup::SharedPtr callback_group_laser;
    rclcpp::CallbackGroup::SharedPtr callback_group_other;
    std::shared_ptr<rclcpp::Node> node;


private:
    void recv_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void map_local_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void map_global_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    QImage rotateMaoWithY(QImage map);
    QPointF m_worldOrigin;

    double m_resolution;
    double m_localresolution;

//**
signals:
    void emitTopicData(QString);
    void emitupdateMap(QImage img);
//    void emitupdatelocalMap(QImage img);
    void emitupdatelocalMap_two(QImage img, robotPose pose);
    void emitupdateglobalMap(QImage img);
    void emitupdaterobotPose(robotPose pose);
    void emitupdatelaserPoints(QPolygonF points);
};

#endif // RCLCOMM_H
