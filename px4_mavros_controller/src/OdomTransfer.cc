/**
 * @file OdomTransfer.cc
 * @brief Transfer a Odometry data to another topic or just change its frame_id, etc
 * Created by Zhaohong Liu on 23-7-17.
*/

// include libs
#include <cstring>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

// global variables
nav_msgs::Odometry odom_pose;
geometry_msgs::PoseStamped pose;
geometry_msgs::Point origin_point;

void odom_pose_cb(const nav_msgs::Odometry::ConstPtr &msg){
    odom_pose = *msg;

    odom_pose.pose.pose.position.z -= origin_point.z;

    pose.pose.position.x = odom_pose.pose.pose.position.x;
    pose.pose.position.y = odom_pose.pose.pose.position.y;
    pose.pose.position.z = odom_pose.pose.pose.position.z;

    pose.pose.orientation.x = odom_pose.pose.pose.orientation.x;
    pose.pose.orientation.y = odom_pose.pose.pose.orientation.y;
    pose.pose.orientation.z = odom_pose.pose.pose.orientation.z;
    pose.pose.orientation.w = odom_pose.pose.pose.orientation.w;
}

void origin_point_sub_cb(const geometry_msgs::Point::ConstPtr &msg){
    origin_point = *msg;
}

int main(int argc, char **argv){
    // ros init
    ros::init(argc, argv, "odom_transfer");
    ros::NodeHandle nodeHandle;

    // init origin point
    origin_point.x = 0.0;
    origin_point.y = 0.0;
    origin_point.z = 0.0;

    // some flags
    bool if_nav_msg_source = false;
    bool if_nav_msg_target = false;
    bool if_nav_force_change_frame = false;
    bool if_geo_msg_target = false;

    std::string nav_msg_source_topic, nav_msg_target_topic, poseStamped_topic;
    std::string target_frame_id;

    // get topic names
    nodeHandle.getParam("odometry_source_topic", nav_msg_source_topic);
    if (!nav_msg_source_topic.empty()){
        if_nav_msg_source = true;
        ROS_INFO("Loaded parameter odometry_source_topic: %s", nav_msg_source_topic.c_str());
    }else{
        ROS_WARN("odometry_source_topic is empty, odometry topic transferring canceled.");
    }

    nodeHandle.getParam("odometry_target_topic", nav_msg_target_topic);
    if (!nav_msg_target_topic.empty()){
        if_nav_msg_target = true;
        ROS_INFO("Loaded parameter odometry_target_topic: %s", nav_msg_target_topic.c_str());
    }else{
        ROS_WARN("odometry_target_topic is empty, odometry topic transferring canceled.");
    }

    nodeHandle.getParam("poseStamped_topic", poseStamped_topic);
    if (!poseStamped_topic.empty()){
        if_geo_msg_target = true;
        ROS_INFO("Loaded parameter poseStamped_topic: %s", poseStamped_topic.c_str());
    }else{
        ROS_WARN("poseStamped_topic is empty, odometry to geo transferring canceled.");
    }

    nodeHandle.getParam("target_frame_id", target_frame_id);
    if (!target_frame_id.empty()){
        if_nav_force_change_frame = true;
        ROS_WARN("Loaded parameter target_frame_id: %s, make sure you want to do this.",
                 target_frame_id.c_str());
    }else{
        ROS_INFO("No force converting of frame_id, ready to start...");
    }

    bool if_nav_solid = if_nav_msg_source && if_nav_msg_target;
    bool if_geo_solid = if_geo_msg_target;

    // mode
    enum MODE{
        ODOM_ONLY,
        ODOM2POSE_ONLY,
        ODOM_AND_POSE,
        NO_MODULE
    };
    MODE m_mode;

    if (if_nav_solid && !if_geo_solid && if_nav_force_change_frame){
        m_mode = ODOM_ONLY;
        ROS_INFO("Published a new nav_msg::Odometry topic: %s", nav_msg_target_topic.c_str());
        ROS_INFO("Frame of %s is: %s", nav_msg_target_topic.c_str(), target_frame_id.c_str());
    }else{
        if (!if_nav_solid && if_geo_solid && if_nav_force_change_frame){
            m_mode = ODOM2POSE_ONLY;
            ROS_INFO("Converted and published a new geometry_msg::PoseStamped topic: %s",
                     poseStamped_topic.c_str());
            ROS_INFO("Frame of %s is: %s", poseStamped_topic.c_str(), target_frame_id.c_str());
        }else if(if_nav_solid && if_geo_solid && if_nav_force_change_frame){
            m_mode = ODOM_AND_POSE;
            ROS_INFO("Published a new nav::Odometry topic: %s", nav_msg_target_topic.c_str());
            ROS_INFO("Frame of %s is %s", nav_msg_target_topic.c_str(), target_frame_id.c_str());
            ROS_INFO("Converted and published a new geometry_msg::PoseStamped topic: %s",
                     poseStamped_topic.c_str());
            ROS_INFO("Frame of %s is %s", poseStamped_topic.c_str(), target_frame_id.c_str());
        }
        else m_mode = NO_MODULE;
    }

    ros::Subscriber odom_pose_sub = nodeHandle.subscribe<nav_msgs::Odometry>(
            nav_msg_source_topic, 100, odom_pose_cb);
    ros::Publisher new_odom_pub = nodeHandle.advertise<nav_msgs::Odometry>(
            nav_msg_target_topic, 100);
    ros::Publisher geo_pose_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>(
            poseStamped_topic, 100);
    ros::Subscriber origin_point_sub = nodeHandle.subscribe<geometry_msgs::Point>(
        "/origin_pos", 10, origin_point_sub_cb);

    // start transferring
    ros::Rate rate(100);
    while (nodeHandle.ok()){
        switch (m_mode) {
            case ODOM_ONLY:
                odom_pose.header.frame_id = target_frame_id;
                new_odom_pub.publish(odom_pose);
                break;
            case ODOM2POSE_ONLY:
                pose.header.frame_id = target_frame_id;
                geo_pose_pub.publish(pose);
                break;
            case ODOM_AND_POSE:
                odom_pose.header.frame_id = target_frame_id;
                pose.header.frame_id = target_frame_id;
                new_odom_pub.publish(odom_pose);
                geo_pose_pub.publish(pose);
                break;
            case NO_MODULE:
                ROS_WARN("Receiving no transferring, shutdown this node...");
                ros::shutdown();
                break;
            default:
                ROS_WARN("Receiving no transferring, shutdown this node...");
                ros::shutdown();
                break;
        }

        ros::spinOnce();
        rate.sleep();
    }
}