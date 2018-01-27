
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include "pcl_pipeline_utils/CompressedPointCloud.h"
#include "pcl_utils.h"


/**
 * Simple class to allow decompression of a point cloud
 * Originally from https://github.com/ChenxiTU/OctreeCompression4Ros
 */
class Decompression {
private:
    
    pcl::io::OctreePointCloudCompression<pcl_utils::PointT>* _PointCloudDecoder;
    pcl::PointCloud<pcl_utils::PointT>::Ptr _pclCloud;
    sensor_msgs::PointCloud2::Ptr _outputMsg;

public:
    Decompression()
        :
        _PointCloudDecoder(new pcl::io::OctreePointCloudCompression<pcl_utils::PointT>),
        _pclCloud(new pcl::PointCloud<pcl_utils::PointT>),
        _outputMsg(new sensor_msgs::PointCloud2)
    {
        // Pass
    };


    ros::Subscriber sub;
    ros::Publisher pub;
    void cloudCallback(const pcl_pipeline_utils::CompressedPointCloud::ConstPtr& cloud_msg);
};


/**
 * Callback that performs the Point Cloud decompression
 */
void Decompression::cloudCallback(const pcl_pipeline_utils::CompressedPointCloud::ConstPtr& msg)
{
    // Stringstream to retrieve compressed point cloud
    std::stringstream compressed_data(msg->data);

    //Decompress the point cloud
    _PointCloudDecoder->decodePointCloud(compressed_data, _pclCloud);

    //Convert back to sensor_msgs::PointCloud2
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2 (*_pclCloud, pcl_pc2);

    pcl_conversions::fromPCL(pcl_pc2, *_outputMsg);

    // Restore the cloud header
    _outputMsg->header = msg->header;
    
    // Pubish the cloud
    pub.publish(_outputMsg);

    float original_size = msg->data.size() / 1024;
    float inflated_size = _outputMsg->data.size() / 1024;
    ROS_DEBUG(
       "Published cloud, original %.2fKiB, inflated %.2fKiB",
       original_size,
       inflated_size
    );
}


/**
 * Main
 */
int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pcl_compression");
    ros::NodeHandle nh, pnh("~");

    // Create our filter
    Decompression MyObj;
    const boost::function< void(const pcl_pipeline_utils::CompressedPointCloud::ConstPtr &)> boundCloudCallback = \
        boost::bind(&Decompression::cloudCallback, &MyObj, _1);

    // Create a ROS subscriber for the input
    MyObj.sub = nh.subscribe<pcl_pipeline_utils::CompressedPointCloud>("/input", 10, boundCloudCallback);

    // Create a ROS publisher for the output
    MyObj.pub = nh.advertise<sensor_msgs::PointCloud2>("/output", 10);

    // Spin
    ros::spin();
}