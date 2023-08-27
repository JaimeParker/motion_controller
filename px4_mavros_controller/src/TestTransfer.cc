//
// Created by hazyparker on 23-8-27.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "mp_test_node");
    ros::NodeHandle nh;

    ros::Publisher publisher = nh.advertise<geometry_msgs::PoseStamped>(
            "/test_pose", 100);

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 1.0;
    pose.pose.position.y = 2.0;
    pose.pose.position.z = 3.0;
    pose.pose.orientation.x = 0.5;

    pose.header.frame_id = "world";
    pose.header.stamp = ros::Time::now();

    while(ros::ok()){
        publisher.publish(pose);

        ros::Rate(100).sleep();
    }
}
