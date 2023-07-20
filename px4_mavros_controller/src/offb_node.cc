/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 * reference: https://docs.px4.io/main/en/ros/mavros_offboard_cpp.html
 * after revise, this exe tested successfully in real test using a 450 mm quad rotor drone
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <vector>

// call back function
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void local_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

inline bool reached(geometry_msgs::PoseStamped& pose,
                    geometry_msgs::PoseStamped& target ){
    return sqrt(pow(pose.pose.position.x - target.pose.position.x, 2)
    + pow(pose.pose.position.y - target.pose.position.y, 2)
    + pow(pose.pose.position.z - target.pose.position.z, 2)) < 0.2;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
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

    int points_num = 0;
    nh.param("point_num", points_num, -1);
    std::vector<std::vector<double>> points(points_num, std::vector<double>(3));
    for (int i = 0; i < points_num; i++)
    {
        nh.param("point" + std::to_string(i) + "_x", points[i][0], -1.0);
        nh.param("point" + std::to_string(i) + "_y", points[i][1], -1.0);
        nh.param("point" + std::to_string(i) + "_z", points[i][2], -1.0);
    }

    //the set-point publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ROS_INFO("fcu connecting...");
        // mind the fcu_url in launch file and gazebo_setup.bash in ~/.bashrc
        // for Pixhawk 4 or 6c firmware, mind the fcu_url in mavros px4.launch
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("fcu connected!");

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = points[0][0];
    pose.pose.position.y = points[0][1];
    pose.pose.position.z = points[0][2];

    //send a few set points before starting
    ROS_INFO("publishing init points");
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
    // flag variable
    // Necessary for real test, enabling to switch to manual control
    bool if_offboard = false;
    bool if_armed = false;
    int iter = 0;

    while(ros::ok()){
        // give the highest priority to manual control (RC controller)
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

        if (reached(current_pose, pose)){
            ROS_INFO("reached way point %s", std::to_string(iter).c_str());
            iter += 1;
            if (iter >= points_num) break;

            pose.pose.position.x = points[iter][0];
            pose.pose.position.y = points[iter][1];
            pose.pose.position.z = points[iter][2];
            for(int i = 100; ros::ok() && i > 0; --i){
                local_pos_pub.publish(pose);
                ros::spinOnce();
                rate.sleep();
            }
        }
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

