/**
 * @file DepthImageTransfer.cc
 * @brief Transfer a sensor_msg::Image to another topic
 * Created by hazy parker on 23-7-18.
*/

#include <cstring>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

sensor_msgs::Image depth;

void depth_sub_cb(const sensor_msgs::Image::ConstPtr &msg){
    depth = *msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "depth_transfer");
    ros::NodeHandle nodeHandle;

    bool if_sensor_source_topic = false;
    bool if_sensor_target_topic = false;
    bool if_sensor_target_frame_id_valid = false;

    std::string sensor_msg_source_topic, sensor_msg_target_topic;
    std::string sensor_msg_target_frame_id;

    nodeHandle.getParam("sensor_msg_source_topic", sensor_msg_source_topic);
    nodeHandle.getParam("sensor_msg_target_topic", sensor_msg_target_topic);
    nodeHandle.getParam("sensor_msg_target_frame_id", sensor_msg_target_frame_id);

    if(!sensor_msg_source_topic.empty()){
        if_sensor_source_topic = true;
        ROS_INFO("Loaded sensor_msgs::Image source topic: %s", sensor_msg_source_topic.c_str());
    }else{
        ROS_WARN("sensor_msgs::Image, source topic invalid");
    }

    if(!sensor_msg_target_topic.empty()){
        if_sensor_target_topic = true;
        ROS_INFO("Loaded sensor_msgs::Image target topic: %s", sensor_msg_target_topic.c_str());
    }else{
        ROS_WARN("sensor_msgs::Image, target topic invalid");
    }

    if(!sensor_msg_target_frame_id.empty()){
        if_sensor_target_frame_id_valid = true;
        ROS_INFO("Loaded sensor_msgs::Image frame_id: %s", sensor_msg_target_frame_id.c_str());
    }else{
        ROS_WARN("No target sensor_msgs::Image frame_id received, please provide one");
    }

    if(!(if_sensor_source_topic && if_sensor_target_topic && if_sensor_target_frame_id_valid)){
        ROS_WARN("Uncompleted information, will shutdown depth_transfer node");
    }

    ros::Subscriber depth_image_sub = nodeHandle.subscribe<sensor_msgs::Image>(
            sensor_msg_source_topic, 100, depth_sub_cb);
    ros::Publisher depth_image_pub = nodeHandle.advertise<sensor_msgs::Image>(
            sensor_msg_target_topic, 100);

    ros::Rate rate(100);
    while(nodeHandle.ok()){
        depth.header.frame_id = sensor_msg_target_frame_id;
        depth_image_pub.publish(depth);

        ros::spinOnce();
        rate.sleep();
    }

}