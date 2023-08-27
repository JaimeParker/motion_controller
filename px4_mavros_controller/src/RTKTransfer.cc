/**
 * @file RTKTransfer.cc
 * @brief get mavros local position, publishing with a new frame
 * Created by Zhaohong Liu on 23-8-26.
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>

geometry_msgs::PoseStamped rtk_pose;  // pose of mavros ENU axis
bool if_rtk_received = false;
double origin_x, origin_y, origin_z;
tf2::Quaternion origin_q;

void rtkPoseSubCB(const geometry_msgs::PoseStamped::ConstPtr &msg){
    rtk_pose = *msg;
    if_rtk_received = true;
}

void getOrigin(std::vector<geometry_msgs::PoseStamped> &m_poses, bool &flag){
    flag = false;

    // FIXME: need a better algorithm to get a robust initial pose

    origin_x = 0;
    origin_y = 0;
    origin_z = 0;
    double q_x, q_y, q_z, q_w;

    for (int i = 0; i < 100; i++){
        origin_x += m_poses[i].pose.position.x;
        origin_y += m_poses[i].pose.position.y;
        origin_z += m_poses[i].pose.position.z;
        q_x += m_poses[i].pose.orientation.x;
        q_y += m_poses[i].pose.orientation.y;
        q_z += m_poses[i].pose.orientation.z;
        q_w += m_poses[i].pose.orientation.w;
    }

    origin_x = origin_x / 100;
    origin_y = origin_y / 100;
    origin_z = origin_z / 100;
    origin_q.setValue(q_x, q_y, q_z, q_w);
    origin_q.normalize();

    flag = true;
}

void setOriginTrans(const geometry_msgs::PoseStamped &msg, geometry_msgs::PoseStamped &pose){
    pose = msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "RTKTransfer");
    ros::NodeHandle nodeHandle;

    ros::Subscriber rtk_pose_sub = nodeHandle.subscribe<geometry_msgs::PoseStamped>(
            "/mavros/local_position/pose", 100, rtkPoseSubCB);
    ros::Publisher origin_trans_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>(
            "/vision_pose", 100);

    while(true){
        if (!if_rtk_received){
            ROS_WARN("warning: no /mavros/local_position/pose data received.");
        }
        else{
            ROS_INFO("/mavros/local_position/pose data received.");
            break;
        }
        ros::spinOnce();
        ros::Rate(1).sleep();
    }

    ROS_INFO("Resolving GPS data.");
    std::vector<geometry_msgs::PoseStamped> rtk_poses;
    bool pose_steady = false;
    int iter = 0;
    while(ros::ok()){
        iter += 1;
        if(!pose_steady && iter <= 100){
            rtk_poses.push_back(rtk_pose);
        }else if(!pose_steady && iter > 100){
            getOrigin(rtk_poses, pose_steady);
            rtk_poses.clear();
            iter = 0;
        }else{
            break;
        }
        ros::spinOnce();
        ros::Rate(100).sleep();
    }
    ROS_INFO("Successfully linked up with GPS data.");

    geometry_msgs::PoseStamped pose_before, pose_after;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    tf2::Quaternion origin_q_inv = origin_q.inverse();
    while(ros::ok()){
        setOriginTrans(rtk_pose, pose_after);
        pose_after.header.frame_id = "world";
        tf::Quaternion q;
        q.setValue(origin_q.x(), origin_q.y(), origin_q.z(), origin_q.w());
        double yaw = tf::getYaw(q);
        yaw = -yaw;
        q.setRPY(0, 0, yaw);
        q.normalize();
        pose_after.pose.orientation.x = q.x();
        pose_after.pose.orientation.y = q.y();
        pose_after.pose.orientation.z = q.z();
        pose_after.pose.orientation.w = q.w();

//        pose_after = buffer.transform(pose_before, "world");
//        tf2::convert(pose_after.pose.orientation, origin_q_inv);
//        double roll, pitch, yaw;
//        tf2::Matrix3x3 mat(origin_q_inv);
//        mat.getRPY(roll, pitch, yaw);

        pose_after.pose.position.x = pose_after.pose.position.x - origin_x;
        pose_after.pose.position.y = pose_after.pose.position.y - origin_y;
        pose_after.pose.position.z = pose_after.pose.position.z - origin_z;
        origin_trans_pub.publish(pose_after);

        ros::spinOnce();
        ros::Rate(100).sleep();
    }
}