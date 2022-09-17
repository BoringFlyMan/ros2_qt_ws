#include "rclcomm.h"


rclcomm::rclcomm()
{
    int argc = 0;
    char **argv = NULL;
    rclcpp::init(argc,argv);
    node = rclcpp::Node::make_shared("ros2_qt_gui_app");
    m_executor = new rclcpp::executors::MultiThreadedExecutor;
    m_executor->add_node(node);
    callback_group_laser = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_other = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto sub_laser_obt = rclcpp::SubscriptionOptions();
    sub_laser_obt.callback_group = callback_group_laser;
    auto sub_other_obt = rclcpp::SubscriptionOptions();
    sub_other_obt.callback_group = callback_group_other;


    _publisher = node->create_publisher<std_msgs::msg::Int32>("ros2_qt_gui_app_publisher", 10);
    _subscription = node->create_subscription<std_msgs::msg::Int32>("ros2_qt_gui_app_publisher", 10, std::bind(&rclcomm::recv_callback, this, std::placeholders::_1),sub_other_obt);
    _map_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(), std::bind(&rclcomm::map_callback, this, std::placeholders::_1),sub_other_obt);
    _map_local_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>("local_costmap/costmap",rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(), std::bind(&rclcomm::map_local_callback, this, std::placeholders::_1),sub_other_obt);
    _map_global_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>("global_costmap/costmap",rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(), std::bind(&rclcomm::map_global_callback, this, std::placeholders::_1),sub_other_obt);
    _laser_sub = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&rclcomm::laser_callback, this, std::placeholders::_1),sub_laser_obt);
    _globalpath_sub = node->create_subscription<nav_msgs::msg::Path>("/plan", 10, std::bind(&rclcomm::globalpath_callback, this, std::placeholders::_1),sub_other_obt);
    _localpath_sub = node->create_subscription<nav_msgs::msg::Path>("/local_plan", 10, std::bind(&rclcomm::localpath_callback, this, std::placeholders::_1),sub_other_obt);
    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    m_transform_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
}

void rclcomm::run(){
    //qDebug()<<"i am run ";
    std_msgs::msg::Int32 pub_msg;
    pub_msg.data = 0;
    rclcpp::WallRate loop_rate(20);
    while(rclcpp::ok()){
        _publisher->publish(pub_msg);
        pub_msg.data++;
//        rclcpp::spin_some(node);
        m_executor->spin_some();
        getRobotPose();
        loop_rate.sleep();
    }
    rclcpp::shutdown();
}

void rclcomm::getRobotPose(){
    try {
        geometry_msgs::msg::TransformStamped transform = m_tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
        geometry_msgs::msg::Quaternion msg_quat = transform.transform.rotation;

        tf2::Quaternion q;
        tf2::fromMsg(msg_quat, q);
        tf2::Matrix3x3 mat(q);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        double x = transform.transform.translation.x;
        double y = transform.transform.translation.y;

        QPointF trans_pose = transWorldPoint2Scene(QPointF(x, y));
        m_currpose.x = trans_pose.x();
        m_currpose.y = trans_pose.y();
        m_currpose.theta = yaw;

        emit emitupdaterobotPose(m_currpose);
//        qDebug()<<"111"<<"x: "<<m_currpose.x<<"y: "<<m_currpose.y;

    } catch (tf2::TransformException &ex) {
        qDebug()<<"robot pose transform error"<<ex.what();
    }
}

void rclcomm::recv_callback(const std_msgs::msg::Int32::SharedPtr msg){
//    qDebug()<<msg->data;
    emit emitTopicData("I am listen the topic data: " + QString::fromStdString(std::to_string(msg->data)));
}



QPointF rclcomm::transWorldPoint2Scene(QPointF point){
    QPointF ret;
    ret.setX(m_worldOrigin.x() + point.x() / m_resolution);
    ret.setY(m_worldOrigin.y() - point.y() / m_resolution);
    return ret;
}

QPointF rclcomm::transScene2WorldPoint(QPointF point){
    QPointF ret;
    ret.setX((point.x() - m_worldOrigin.x()) * m_resolution);
    ret.setY(-1 * (point.y() - m_worldOrigin.y()) * m_resolution);
    return ret;
}

QImage rclcomm::rotateMaoWithY(QImage map){
    QImage res = map;
    for(int x = 0; x < map.width(); x++){
        for(int y = 0; y < map.height(); y++){
            res.setPixelColor(x, map.height() - y -1, map.pixelColor(x, y));
        }
    }
    return res;
}

void rclcomm::localpath_callback(const nav_msgs::msg::Path::SharedPtr msg){
    try {
        geometry_msgs::msg::PointStamped point_map_frame;
        geometry_msgs::msg::PointStamped point_odom_frame;

        QPolygonF points;
        for(int i = 0; i < msg->poses.size(); i++){
            point_odom_frame.point.x = msg->poses[i].pose.position.x;
            point_odom_frame.point.y = msg->poses[i].pose.position.y;
            point_odom_frame.header.frame_id = msg->header.frame_id;
            m_tf_buffer->transform(point_odom_frame, point_map_frame, "map");
            QPointF point;
            point = transWorldPoint2Scene(QPointF(point_map_frame.point.x, point_map_frame.point.y));
            points.push_back(point);
        }
        emit emituodatelocalPath(points);
    } catch (tf2::TransformException &ex) {
        qDebug()<<"local_plan transform error"<<ex.what();
    }
}

//plan global_path
void rclcomm::globalpath_callback(const nav_msgs::msg::Path::SharedPtr msg){
    QPolygonF points;
    for(int i = 0; i < msg->poses.size(); i++){
        QPointF point;
        point.setX(msg->poses[i].pose.position.x);
        point.setY(msg->poses[i].pose.position.y);
        point = transWorldPoint2Scene(point);
        points.push_back(point);
    }
    emit emituodateglobalPath(points);
}

//GlobalMap 
void rclcomm::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    double origin_x = msg->info.origin.position.x;
    double origin_y = msg->info.origin.position.y;
    m_resolution = msg->info.resolution;
    qDebug()<<"receive topic /map.";
    int width = msg->info.width;
    int height = msg->info.height;
    QImage map_image(width, height, QImage::Format_RGB32);
    for(int i = 0; i < msg->data.size(); i++){
        int x = i % width;
        int y = int(i / width);
        QColor color;
        if(msg->data[i]==100){
            color = Qt::black;
        }else if(msg->data[i]==0){
            color = Qt::white;
        }else{
            color = Qt::gray;
        }
        map_image.setPixel(x,y,qRgb(color.red(),color.green(),color.blue()));
    }

//    map_image.save("/home/hg/map.png");
    QImage rotate_map = rotateMaoWithY(map_image);
//    rotate_map.save("/home/hg/rotate_map.png");
    emit emitupdateMap(rotate_map);
//    qDebug()<<"test1";

    double trans_origin_x = origin_x;
    double trans_origin_y = origin_y + height * m_resolution;
    qDebug()<<origin_y;

    m_worldOrigin.setX(fabs(trans_origin_x / m_resolution));
    m_worldOrigin.setY(fabs(trans_origin_y / m_resolution));
}

//LocalcostMap
void rclcomm::map_local_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    double origin_x = msg->info.origin.position.x;
    double origin_y = msg->info.origin.position.y;
    m_localresolution = msg->info.resolution;
    int width = msg->info.width;
    int height = msg->info.height;
//    qDebug()<<"3333"<<origin_y;
    tf2::Quaternion q;
    tf2::fromMsg(msg->info.origin.orientation, q);
    tf2::Matrix3x3 mat(q);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    double origin_theta = yaw;

    QImage map_image(width, height, QImage::Format_ARGB32);
    for(int i = 0; i < msg->data.size(); i++){
        int x = i % width;
        int y = int(i / width);
        QColor color;
        int data = msg->data[i];
        if(data >=100){
            color.setRgb(0xff, 00, 0xff);
        }else if(data >= 90 && data < 100){
            color.setRgb(0x66, 0xff, 0xff);
        }else if(data >= 70 && data < 90){
            color.setRgb(0xff, 0x00, 0x33);
        }else if(data >= 60 && data < 70){
            color.setRgb(0xbe, 0x28, 0x1a);
        }else if(data >= 50 && data < 60){
            color.setRgb(0xBE, 0x1f, 0x58);
        }else if(data >= 40 && data < 50){
            color.setRgb(0xBE, 0x25, 0x76);
        }else if(data >= 30 && data < 40){
            color.setRgb(0xBE, 0x2A, 0x99);
        }else if(data >= 20 && data < 30){
            color.setRgb(0xBE, 0x35, 0xB3);
        }else if(data >= 10 && data < 20){
            color.setRgb(0xB0, 0x3C, 0xBE);
        }else{
//            color.setRgba(0xB0, 0x3C, 0xBE, 0);
            color = Qt::transparent;
        }
        map_image.setPixelColor(x,y,color);
    }
//    //map_image.save("/home/hg/localmap.png");
        QImage rotate_map = rotateMaoWithY(map_image);
//    //rotate_map.save("/home/hg/localmap2.png");
//      emit emitupdatelocalMap(rotate_map);

//    qDebug()<<"succesfully receive localcostmap";
    try {
        geometry_msgs::msg::PoseStamped pose_map_frame;
        geometry_msgs::msg::PoseStamped pose_odom_frame;
        pose_odom_frame.pose.position.x = origin_x;
        pose_odom_frame.pose.position.y = origin_y;
        q.setRPY(0, 0, origin_theta);
        pose_odom_frame.pose.orientation = tf2::toMsg(q);
        pose_odom_frame.header.frame_id = msg->header.frame_id;

        m_tf_buffer->transform(pose_odom_frame, pose_map_frame, "map");
        tf2::fromMsg(pose_map_frame.pose.orientation, q);
        tf2::Matrix3x3 mat(q);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        QPointF scene_origin = transWorldPoint2Scene(QPointF(pose_map_frame.pose.position.x,pose_map_frame.pose.position.y));
        robotPose localcostmapPose;
        localcostmapPose.x = scene_origin.x();
        localcostmapPose.y = scene_origin.y();
        localcostmapPose.theta = yaw;
        emit emitupdatelocalMap_two(rotate_map, localcostmapPose);


    } catch (tf2::TransformException &ex) {
        qDebug()<<"localcostmap transform error"<<ex.what();
    }

}

//GlobalcostMap
void rclcomm::map_global_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    double origin_x = msg->info.origin.position.x;
    double origin_y = msg->info.origin.position.y;
    m_localresolution = msg->info.resolution;
    int width = msg->info.width;
    int height = msg->info.height;
//    qDebug()<<"3333"<<origin_y;
    QImage map_image(width, height, QImage::Format_ARGB32);
    for(int i = 0; i < msg->data.size(); i++){
        int x = i % width;
        int y = int(i / width);
        QColor color;
        int data = msg->data[i];
        if(data >=100){
            color.setRgb(0xff, 00, 0xff);
        }else if(data >= 90 && data < 100){
            color.setRgb(0x66, 0xff, 0xff);
        }else if(data >= 70 && data < 90){
            color.setRgb(0xff, 0x00, 0x33);
        }else if(data >= 60 && data < 70){
            color.setRgb(0xbe, 0x28, 0x1a);
        }else if(data >= 50 && data < 60){
            color.setRgb(0xBE, 0x1f, 0x58);
        }else if(data >= 40 && data < 50){
            color.setRgb(0xBE, 0x25, 0x76);
        }else if(data >= 30 && data < 40){
            color.setRgb(0xBE, 0x2A, 0x99);
        }else if(data >= 20 && data < 30){
            color.setRgb(0xBE, 0x35, 0xB3);
        }else if(data >= 10 && data < 20){
            color.setRgb(0xB0, 0x3C, 0xBE);
        }else{
//            color.setRgba(0xB0, 0x3C, 0xBE, 0);
            color = Qt::transparent;
        }
        map_image.setPixelColor(x,y,color);
    }
//    map_image.save("/home/hg/localmap.png");
    QImage rotate_map = rotateMaoWithY(map_image);
//    rotate_map.save("/home/hg/localmap2.png");
    emit emitupdateglobalMap(rotate_map);

    qDebug()<<"succesfully receive globalcostmap";
}

//Laser_Scan
void rclcomm::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
//    qDebug()<<"sucessfullr receive /scan";
    double angle_min = msg->angle_min;
    double angle_max = msg->angle_max;
    double angle_increment = msg->angle_increment;
    try {
        //one
        geometry_msgs::msg::PointStamped point_map_frame;
        geometry_msgs::msg::PointStamped point_laser_frame;
        //two
//        geometry_msgs::msg::TransformStamped laser_transform = m_tf_buffer->lookupTransform("map", "base_scan", tf2::TimePointZero);

        QPolygonF trans_points;
        for(int i = 0; i < msg->ranges.size(); i++){
            double theta = i * angle_increment + angle_min;
            double x = msg->ranges[i] * cos(theta);
            double y = msg->ranges[i] * sin(theta);
            point_laser_frame.point.x = x;
            point_laser_frame.point.y = y;
            point_laser_frame.header.frame_id = msg->header.frame_id;

            m_tf_buffer->transform(point_laser_frame, point_map_frame, "map");
            //two
//            tf2::doTransform(point_laser_frame, point_map_frame, laser_transform);

//            qDebug()<<"before: "<<x<<y<<"back: "<<point_map_frame.point.x<<point_map_frame.point.y;
            QPointF trans_laser_point;
            trans_laser_point.setX(point_map_frame.point.x);
            trans_laser_point.setY(point_map_frame.point.y);
            trans_laser_point = transWorldPoint2Scene(trans_laser_point);

            trans_points.push_back(trans_laser_point);
        }
        emit emitupdatelaserPoints(trans_points);

    } catch (tf2::TransformException &ex) {
        qDebug()<<"laser transform error"<<ex.what();
    }

}
