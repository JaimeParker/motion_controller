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

// global variables
geometry_msgs::PoseStamped vision_pose;

// callback function for subscribing /camera/odom/sample
void t265_odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    nav_msgs::Odometry odom_pose;
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

int main(int argc, char **argv){
    ros::init(argc, argv, "T265OdomTransfer");
    ros::NodeHandle nodeHandle;

    // TODO: using tf2 is a more standard way to do this, we'll do it later
    tf::TransformListener listener;

    ros::Subscriber camera_odom_sub = nodeHandle.subscribe<nav_msgs::Odometry>(
            "/camera/odom/sample", 1000, t265_odom_cb);
    ros::Publisher vision_pose_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>(
            "mavros/vision_pose/pose", 100);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        vision_pose_pub.publish(vision_pose);
        rate.sleep();
    }

    return 0;
}
