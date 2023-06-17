/**
 * @file D435iPCL2Octomap.cc
 * @brief Transfer sensor_msgs/PointCloud2 to octomap
 * Created by hazy parker on 23-6-17.
*/

// c++ lib header
// ros and 3rdParty header
#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_ros/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap_msgs/conversions.h>

sensor_msgs::PointCloud2 PCL2_msg;

// PCL2 sub callback
void D435i_PCL2_cb(const sensor_msgs::PointCloud2::ConstPtr &msg){
    PCL2_msg = *msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "D435iPCL2Octomap");
    ros::NodeHandle nodeHandle;

    ros::Subscriber PCL2_sub = nodeHandle.subscribe(
            "/camera/depth/color/points", 100, D435i_PCL2_cb);
    ros::Publisher octomap_pub = nodeHandle.advertise<octomap_msgs::Octomap>(
            "/m_octomap", 100);

    octomap_msgs::Octomap octomap_msg;
    octomap::Pointcloud octomapCloud;
    octomap::OcTree *octomap_full = nullptr;

    while (ros::ok()){
        ros::spinOnce();
        // FIXME: how to transfer octomap::Pointcloud to octomap_msgs::Octomap
        octomap::pointCloud2ToOctomap(PCL2_msg, octomapCloud);
        octomap_msgs::binaryMapToMsg(*octomap_full, octomap_msg);

    }

    return 0;
}
