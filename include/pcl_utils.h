/**
 * ROS <> PCL Message Conversion helper methods
 * Based on https://github.com/OSUrobotics/barrett_wam_grasp_capture_host/blob/master/grasp_manager/src/pcl_to_ros.cpp
 */

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

namespace pcl_utils
{

    // Typedef for the kind of points we can compress / decompress
    // NB: PointXYZRGBNormal doesn't seem to be supported by pcl/compression/octree_pointcloud_compression
    typedef pcl::PointXYZRGB PointT;


    /**
     * Simple struct to store compression profiles
     */
    struct ConfigurationProfile
    {
        bool showStatistics;
        double pointResolution;
        float octreeResolution;
        bool doVoxelGridDownDownSampling;
        unsigned int iFrameRate;
        bool doColorEncoding;
        unsigned int colorBitResolution;

        ConfigurationProfile()
        {
            showStatistics = false;
            pointResolution = 0.03;
            octreeResolution = 0.03f;
            doVoxelGridDownDownSampling = true;
            iFrameRate = 100;
            doColorEncoding = true;
            colorBitResolution = 4;
        }
    };


    /**
     * Helper function to convert from sensor_msgs/PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr fromROSMsg(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
        return cloud;
    };

    /**
     * Helper function to convert from sensor_msgs/PointCloud2 to pcl::PointCloud<pcl::PointXYZRGB>
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fromROSMsgRGB(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
        return cloud;
    };

    /**
     * Helper function to convert from sensor_msgs/PointCloud2 to pcl::PointCloud<pcl::PointXYZRGBNormal>
     */
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr fromROSMsgRGBNormal(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
        return cloud;
    };

    /**
     * Helper function to convert from pcl::PointCloud<pcl::PointXYZ> to sensor_msgs/PointCloud2
     */
    sensor_msgs::PointCloud2::Ptr toROSMsg(pcl::PointCloud<pcl::PointXYZ>& cloud)
    {
        sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(cloud, *msg);
        return msg;
    };

    /**
     * Helper function to convert from pcl::PointCloud<pcl::PointNormal> to sensor_msgs/PointCloud2
     */
    sensor_msgs::PointCloud2::Ptr toROSMsg(pcl::PointCloud<pcl::PointNormal>& cloud)
    {
        sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(cloud, *msg);
        return msg;
    };

    /**
     * Helper function to convert from pcl::PointCloud<pcl::PointXYZRGBNormal> to sensor_msgs/PointCloud2
     */
    sensor_msgs::PointCloud2::Ptr toROSMsg(pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud)
    {
        sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(cloud, *msg);
        return msg;
    };

};
