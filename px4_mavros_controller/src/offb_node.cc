/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 * reference: https://docs.px4.io/main/en/ros/mavros_offboard_cpp.html
 */

// add some information output

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

// call back function
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void local_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

int main(int argc, char **argv)
{
    // init ros node
    ros::init(argc, argv, "offb_node");
    // create node handler
    ros::NodeHandle nh;

    // define subscriber and publisher
    // define a service client
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose",10,local_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the set-point publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ROS_INFO("fcu connecting...");
        // mind the fcu_url in launch file and gazebo_setup.bash in ~/.bashrc
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("fcu connected!");

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;

    //send a few set points before starting
    for(int i = 200; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    bool if_offboard = false;
    bool if_armed = false;

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" && !if_offboard &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Switched to Offboard mode!");
                if_offboard = true;
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed && !if_armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed!");
                    if_armed = true;
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();

        double dist = sqrt(
                pow(current_pose.pose.position.x - pose.pose.position.x, 2) +
                pow(current_pose.pose.position.y - pose.pose.position.y, 2) +
                pow(current_pose.pose.position.z - pose.pose.position.z, 2));
        if (dist <= 0.2){
            ROS_INFO("Takeoff Successfully!");
            break;
        }
        rate.sleep();
    }

    // hover
    ROS_INFO("Hovering...");
    for(int i = 200; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // waypoint
    ROS_INFO("Moving to target position...");
    pose.pose.position.x = 0.5;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;
    while (ros::ok()){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        double dist = sqrt(
                pow(current_pose.pose.position.x - pose.pose.position.x, 2) +
                pow(current_pose.pose.position.y - pose.pose.position.y, 2) +
                pow(current_pose.pose.position.z - pose.pose.position.z, 2));
        if (dist <= 0.2){
            ROS_INFO("Reached target position!");
            break;
        }
        rate.sleep();
    }

    // hover
    ROS_INFO("Hovering...");
    for(int i = 200; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // proceed a landing process
    ROS_INFO("Proceed landing...");
    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";
    while (ros::ok()){
        if( current_state.mode != "AUTO.LAND" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(land_set_mode) &&
                land_set_mode.response.mode_sent){
                ROS_INFO("PX4 command Landing enabled!");
            }
            last_request = ros::Time::now();
        }
        if(!current_state.armed){
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

