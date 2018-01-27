/**
 * ROS <> PCL Message Conversion helper methods
 * Based on https://github.com/OSUrobotics/barrett_wam_grasp_capture_host/blob/master/grasp_manager/src/pcl_to_ros.cpp
 */

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

namespace ros_msg_convert
{
    /**
     * Helper function to convert from sensor_msgs/PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr fromROSMsg(const sensor_msgs::PointCloud2::ConstPtr& msg);

    /**
     * Helper function to convert from sensor_msgs/PointCloud2 to pcl::PointCloud<pcl::PointXYZRGB>
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fromROSMsgRGB(const sensor_msgs::PointCloud2::ConstPtr& msg);

    /**
     * Helper function to convert from pcl::PointCloud<pcl::PointXYZ> to sensor_msgs/PointCloud2
     */
    sensor_msgs::PointCloud2::Ptr toROSMsg(pcl::PointCloud<pcl::PointXYZ>& cloud);

    /**
     * Helper function to convert from pcl::PointCloud<pcl::PointNormal> to sensor_msgs/PointCloud2
     */
    sensor_msgs::PointCloud2::Ptr toROSMsg(pcl::PointCloud<pcl::PointNormal>& cloud);

    /**
     * Helper function to convert from pcl::PointCloud<pcl::PointXYZRGBNormal> to sensor_msgs/PointCloud2
     */
    sensor_msgs::PointCloud2::Ptr toROSMsg(pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud);
};
