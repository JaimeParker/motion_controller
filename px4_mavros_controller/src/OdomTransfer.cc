/**
 * @file OdomTransfer.cc
 * @brief Transfer a Odometry data to another topic or just change its frame_id, etc
 * Created by hazy parker on 23-7-17.
*/

// include libs
#include <cstring>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

// global variables
nav_msgs::Odometry odom_pose;
geometry_msgs::PoseStamped geo_pose;

int main(int argc, char **argv){
    // ros init
    ros::init(argc, argv, "odom_transfer");
    ros::NodeHandle nodeHandle;

    // some flags
    bool if_nav_msg_source = false;
    bool if_nav_msg_target = false;
    bool if_nav_force_change_frame = false;
    bool if_geo_msg_target = false;

    std::string nav_msg_source_topic, nav_msg_target_topic, geo_msg_target_topic;
    std::string target_frame_id;

    // get topic names
    if (nodeHandle.getParam("odometry_source_topic", nav_msg_source_topic)){
        if (!nav_msg_source_topic.empty()) if_nav_msg_source = true;
        ROS_INFO("Loaded parameter odometry_source_topic: %s", nav_msg_source_topic.c_str());
    }else{
        ROS_WARN("odometry_source_topic is empty, odometry topic transferring canceled.");
    }

    if (nodeHandle.getParam("odometry_target_topic", nav_msg_target_topic)){
        if (!nav_msg_target_topic.empty()) if_nav_msg_target = true;
        ROS_INFO("Loaded parameter odometry_target_topic: %s", nav_msg_target_topic.c_str());
    }else{
        ROS_WARN("odometry_target_topic is empty, odometry topic transferring canceled.");
    }

    if (nodeHandle.getParam("odometry_target_topic", geo_msg_target_topic)){
        if (!geo_msg_target_topic.empty()) if_geo_msg_target = true;
        ROS_INFO("Loaded parameter pose_stamp_target_topic: %s", geo_msg_target_topic.c_str());
    }else{
        ROS_WARN("pose_stamp_target_topic is empty, odometry to geo transferring canceled.");
    }

    if (nodeHandle.getParam("nav_target_frame_id", target_frame_id)){
        if (!target_frame_id.empty()) if_nav_force_change_frame = true;
        ROS_WARN("Loaded parameter nav_target_frame_id: %s, make sure you want to do this.",
                 target_frame_id.c_str());
    }else{
        ROS_INFO("No force converting of nav_frame_id, ready to start...");
    }

    bool if_nav_solid = if_nav_msg_source && if_nav_msg_target;

    // mode
    enum MODE{
        FULL,
        FULL_FRAME,
        SOURCE_FRAME,
        DEFAULT
    };

}