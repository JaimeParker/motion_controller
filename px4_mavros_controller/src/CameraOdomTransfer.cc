/**
 * @file T265OdomTransfer.cc
 * @brief Transform pose from T265 odom to pose in map frame,
 * then send it to topic that PX4 needed for external estimation, such as
 * /mavros/vision_pose/pose and /mavros/odometry/out
 * Created by hazy parker on 23-6-16.
 */

// c++ lib header

// ros and 3rdParty header
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Image.h>

// global variables
geometry_msgs::PoseStamped vision_pose;
nav_msgs::Odometry odom_pose;
sensor_msgs::Image depth;

// callback function for subscribing /camera/odom/sample
void t265_odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    odom_pose = *msg;

    vision_pose.header.frame_id = "map";

    vision_pose.pose.position.x = odom_pose.pose.pose.position.x;
    vision_pose.pose.position.y = odom_pose.pose.pose.position.y;
    vision_pose.pose.position.z = odom_pose.pose.pose.position.z;

    vision_pose.pose.orientation.x = odom_pose.pose.pose.orientation.x;
    vision_pose.pose.orientation.y = odom_pose.pose.pose.orientation.y;
    vision_pose.pose.orientation.z = odom_pose.pose.pose.orientation.z;
    vision_pose.pose.orientation.w = odom_pose.pose.pose.orientation.w;
}

void d400_depth_cb(const sensor_msgs::Image::ConstPtr &imagePtr){
    depth = *imagePtr;
    depth.header.frame_id = "world";
}

int main(int argc, char **argv){
    ros::init(argc, argv, "camera_odom_transfer");
    ros::NodeHandle nodeHandle;

    // TODO: using tf2 is a more standard way to do this, we'll do it later
    tf::TransformListener listener;
    tf::StampedTransform transform;

    std::string camera_odom_topic = "/camera/odom/sample";
    if (nodeHandle.getParam("camera_odom_topic", camera_odom_topic)){
        ROS_INFO("Loaded param, camera_odom_topic: %s", camera_odom_topic.c_str());
    }else{
        ROS_INFO("Using default, camera_odom_topic: %s", camera_odom_topic.c_str());
    }

    std::string source_frame_id = "/camera_link";
    if (nodeHandle.getParam("source_frame_id", source_frame_id)){
        ROS_INFO("Loaded param, source_frame_id: %s", source_frame_id.c_str());
    }else{
        ROS_INFO("Using default, source_frame_id: %s", source_frame_id.c_str());
    }

    std::string target_frame_id = "/camera_odom_frame";
    if (nodeHandle.getParam("target_frame_id", target_frame_id)){
        ROS_INFO("Loaded param, target_frame_id: %s", target_frame_id.c_str());
    }else{
        ROS_INFO("Using default, target_frame_id: %s", target_frame_id.c_str());
    }

    std::string vision_pose_topic = "/mavros/vision_pose/pose";
    if (nodeHandle.getParam("vision_pose_topic", vision_pose_topic)){
        ROS_INFO("Loaded param, vision_pose_topic: %s", vision_pose_topic.c_str());
    }else{
        ROS_INFO("Using default, vision_pose_topic: %s", vision_pose_topic.c_str());
    }

    ros::Subscriber camera_odom_sub = nodeHandle.subscribe<nav_msgs::Odometry>(
            camera_odom_topic, 100, t265_odom_cb);
    ros::Publisher vision_pose_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>(
            vision_pose_topic, 100);
    ros::Publisher odom4planner_pub = nodeHandle.advertise<nav_msgs::Odometry>(
            "/camera_odom_planner", 100);
    ros::Subscriber d400_depth_sub = nodeHandle.subscribe<sensor_msgs::Image>(
            "/d400/depth/image_rect_raw", 10, d400_depth_cb);
    ros::Publisher d400_depth_pub = nodeHandle.advertise<sensor_msgs::Image>(
            "/depth_planner", 10);

//    listener.waitForTransform(target_frame_id, source_frame_id,
//                              ros::Time::now(), ros::Duration(2.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    auto tf_transform = tf::Transform(q, tf::Vector3(0, 0, 0));
    auto stampTransform = tf::StampedTransform(tf_transform, ros::Time::now(),
                                               source_frame_id, target_frame_id);
    tf::TransformBroadcaster transformBroadcaster;

    ros::Rate rate(100);
    ROS_INFO("Publishing topic from camera odom pose to vision pose...");
    while (ros::ok()) {
//        ros::Time now = ros::Time(0);
//        try{
//            listener.lookupTransform(target_frame_id, source_frame_id,
//                                     now, transform);
//        } catch (tf::TransformException &exception){
//            ROS_WARN("%s", exception.what());
//            ros::Duration(1.0).sleep();
//        }

        ros::spinOnce();
//        transformBroadcaster.sendTransform(stampTransform);
        vision_pose.header.frame_id = target_frame_id;
        vision_pose.header.frame_id = "world";
        vision_pose_pub.publish(vision_pose);

        odom_pose.header.frame_id = "world";
        odom4planner_pub.publish(odom_pose);

        d400_depth_pub.publish(depth);

        rate.sleep();
    }

    return 0;
}
