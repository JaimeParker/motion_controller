/**
 * @file PoseTransfer.cc
 * @brief Transfer a PoseStamped data to another topic or just change its frame_id, etc
 * Created by Zhaohong Liu on 23-7-17.
*/

// include libs
#include <cstring>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

// global variables
geometry_msgs::PoseStamped pose;
geometry_msgs::Point origin_point;

// callback functions
void pose_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    pose = *msg;

    // only transfer z coordinate
    pose.pose.position.z -= origin_point.z;
}

void origin_point_sub_cb(const geometry_msgs::Point::ConstPtr &msg){
    origin_point = *msg;
}

// main function, ros node
int main(int argc, char **argv){
    // ros init
    ros::init(argc, argv, "pose_transfer");
    ros::NodeHandle nodeHandle;

    // init origin point
    origin_point.x = 0.0;
    origin_point.y = 0.0;
    origin_point.z = 0.0;

    // some flags
    bool if_geo_msg_source = false;
    bool if_geo_msg_target = false;
    bool if_geo_force_change_frame = false;

    // topic names
    std::string geo_msg_source_topic, geo_msg_target_topic;
    std::string geo_target_frame_id;

    // get topic names
    nodeHandle.getParam("pose_stamp_source_topic", geo_msg_source_topic);
    if (!geo_msg_source_topic.empty()){
        if_geo_msg_source = true;
        ROS_INFO("Loaded parameter pose_stamp_source_topic: %s", geo_msg_source_topic.c_str());
    }else{
        ROS_WARN("pose_stamp_source_topic is empty, PoseStamped topic transferring canceled");
    }

    nodeHandle.getParam("pose_stamp_target_topic", geo_msg_target_topic);
    if (!geo_msg_target_topic.empty()){
        if_geo_msg_target = true;
        ROS_INFO("Loaded parameter pose_stamp_target_topic: %s", geo_msg_target_topic.c_str());
    }else{
        ROS_WARN("pose_stamp_target_topic is empty, PoseStamped topic transferring canceled");
    }

    nodeHandle.getParam("pose_stamp_target_frame_id", geo_target_frame_id);
    if (!geo_target_frame_id.empty()){
        if_geo_force_change_frame = true;
        ROS_WARN("Loaded parameter frame_id of PoseStamped: %s, make sure you want to do this",
                 geo_target_frame_id.c_str());
    }else{
        ROS_INFO("No force converting of geo_frame_id, ready to start...");
    }

    bool if_geo_solid = if_geo_msg_source && if_geo_msg_target;
    // mode
    enum MODE{
        FULL,
        FULL_FRAME,
        DEFAULT
    };
    MODE m_mode = DEFAULT;

    if (if_geo_force_change_frame){
        if (if_geo_solid) m_mode = FULL_FRAME;
    }else{
        if (if_geo_solid) m_mode = FULL;
    }

    ros::Subscriber pose_sub = nodeHandle.subscribe<geometry_msgs::PoseStamped>(
            geo_msg_source_topic, 100, pose_sub_cb);
    ros::Publisher pose_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>(
            geo_msg_target_topic, 100);
    ros::Subscriber origin_point_sub = nodeHandle.subscribe<geometry_msgs::Point>(
            "/origin_pos", 10, origin_point_sub_cb);

    switch (m_mode) {
        case FULL:
            ROS_INFO("Published a new geometry_msg::PoseStamped topic: %s",
                     geo_msg_target_topic.c_str());
            break;
        case FULL_FRAME:
            ROS_INFO("Published a new geometry_msg::PoseStamped topic: %s",
                     geo_msg_target_topic.c_str());
            ROS_INFO("%s frame id changed to %s",
                     geo_msg_target_topic.c_str(), geo_target_frame_id.c_str());
            break;
        default:
            ROS_WARN("Uncompleted information in pose_transfer node");
            break;
    }

    // start transferring
    ros::Rate rate(100);
    while (nodeHandle.ok()){
        switch (m_mode) {
            case FULL:
                pose_pub.publish(pose);
                break;
            case FULL_FRAME:
                pose.header.frame_id = geo_target_frame_id;
                pose_pub.publish(pose);
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