/**
 * @file T265Vision2Mavros.cc
 * @brief Transform frame of T265, from camera frame to odometer
 * Created by hazy parker on 23-6-15.
 */

// c++ lib header
#include <cmath>
// ros and 3rdParty header
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <opencv2/opencv.hpp>

template<typename T>
T getParam(const std::string &param, const T &missing_value){
    T value;

    if (ros::param::get(param, value)){
        return value;
    }else{
        ROS_WARN_STREAM("Failed to load param " << param);
        return missing_value;
    }
}

int main(int argc, char **argv){
    // ros config
    ros::init(argc, argv, "T265FrameTransfer");
    ros::NodeHandle nodeHandle;

    // frame id definition
    std::string T265_odom_frame_id = "/camera_odom_frame";
    std::string base_link_id = "/base_link";

    // read yaml file
    std::string cfg_path = "./src/motion_controller/px4_mavros_controller/config/t265.yaml";
    cv::FileStorage config(cfg_path, cv::FileStorage::READ);
    if (!config.isOpened()){
        std::cerr << "Failed to open T265 config file, check the file name and path." << std::endl;
        return -1;
    }

    // get params
    double offset_x, offset_y, offset_z;
    config["cam_x_offset"] >> offset_x;
    config["cam_y_offset"] >> offset_y;
    config["cam_z_offset"] >> offset_z;
    double pitch, roll, yaw;
    config["cam_pitch"] >> pitch;
    config["cam_roll"] >> roll;
    config["cam_yaw"] >> yaw;

    // close file
    config.release();

    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    tf::TransformBroadcaster transformBroadcaster;
    auto transform = tf::Transform(q, tf::Vector3(offset_x, offset_y, offset_z));
    auto stampTransform = tf::StampedTransform(transform,
                                               ros::Time::now(),
                                               T265_odom_frame_id,
                                               base_link_id);
    // FIXME: TF_REPEATED_DATA ignoring data with redundant timestamp for frame camera_odom_frame
    //  at time 1686923725.606676 according to authority unknown_publisher

    ros::Rate rate(100);
    while (ros::ok()){
        transformBroadcaster.sendTransform(stampTransform);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}