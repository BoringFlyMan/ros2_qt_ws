#include "rclcomm.h"

rclcomm::rclcomm()
{
    int argc = 0;
    char **argv = NULL;
    rclcpp::init(argc,argv);
    node = rclcpp::Node::make_shared("ros2_qt_demo");
    _publisher = node->create_publisher<std_msgs::msg::Int32>("ros2_qt_demo_publisher", 10);
    _subscription = node->create_subscription<std_msgs::msg::Int32>("ros2_qt_demo_publisher", 10, std::bind(&rclcomm::recv_callback, this, std::placeholders::_1));

}

void rclcomm::run(){
    //qDebug()<<"i am run ";
    std_msgs::msg::Int32 pub_msg;
    pub_msg.data = 0;
    rclcpp::WallRate loop_rate(1);
    while(rclcpp::ok()){
        _publisher->publish(pub_msg);
        pub_msg.data++;
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
}

void rclcomm::recv_callback(const std_msgs::msg::Int32::SharedPtr msg){
    qDebug()<<msg->data;
    emit emitTopicData("I am listen the topic data: " + QString::fromStdString(std::to_string(msg->data)));


}
