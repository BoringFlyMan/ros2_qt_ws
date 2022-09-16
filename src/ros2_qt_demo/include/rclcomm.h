#ifndef RCLCOMM_H
#define RCLCOMM_H

#include <QObject>
#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <QDebug>
#include <std_msgs/msg/int32.hpp>

class rclcomm :public QThread
{
    Q_OBJECT
public:
    rclcomm();
    void run() override;
private:
   void recv_callback(const std_msgs::msg::Int32::SharedPtr msg);
private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _publisher;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr _subscription;
    std::shared_ptr<rclcpp::Node> node;
//**
signals:
    void emitTopicData(QString);
};

#endif // RCLCOMM_H
