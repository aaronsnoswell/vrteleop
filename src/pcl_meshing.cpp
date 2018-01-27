
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

#include "pcl_utils.h"


/**
 * Simple class to allow meshing of a point cloud
 */
class Meshing
{
public:
    
    enum EMeshMethod
    {
        GreedyProjection,
        Poisson,
        MarchingCubesRBF,
        MarchingCubesHoppe,
        ConvexHull
    };

    Meshing(EMeshMethod method=GreedyProjection)
        :
        _method(method),
        _pclCloud(new pcl::PointCloud<pcl_utils::PointT>)
    {
        // Pass
    };


    ros::Subscriber sub;
    ros::Publisher pub;
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

private:
    EMeshMethod _method;
    pcl::PointCloud<pcl_utils::PointT>::Ptr _pclCloud;
    //pcl_pipeline_utils::CompressedPointCloud _outputMsg;
};


/**
 * Callback that performs the Point Cloud compression
 */
void Meshing::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if(pub.getNumSubscribers())
    {
        // Convert from sensor_msg::PointCloud2 to pcl::PointCloud<pcl_utils::PointT>
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *_pclCloud);

        switch(_method)
        {
            case GreedyProjection:
            {
                
                break;
            }
            case Poisson:
            {
                ROS_WARN(
                    "Poisson meshing method not implemented yet"
                );
                break;
            }
            case MarchingCubesRBF:
            {
                ROS_WARN(
                    "Marching Cubes RBF meshing method not implemented yet"
                );
                break;
            }
            case MarchingCubesHoppe:
            {
                ROS_WARN(
                    "Marching Cubes Hoppe meshing method not implemented yet"
                );
                break;
            }
            case ConvexHull:
            {
                ROS_WARN(
                    "Convex Hull meshing method not implemented yet"
                );
                break;
            }
            default:
            {
                ROS_WARN(
                    "Unknown meshing method requested"
                );
            }
        }

    }
    else
    {
        ROS_INFO(
            "Received input cloud but there are no subscribers - not publishing"
        );
    }
}


/**
 * Main
 */
int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pcl_meshing");
    ros::NodeHandle nh, pnh("~");

    // Create our filter
    Meshing MyObj;
    const boost::function< void(const sensor_msgs::PointCloud2ConstPtr &)> boundCloudCallback = \
        boost::bind(&Meshing::cloudCallback, &MyObj, _1);

    // Create a ROS subscriber for the input
    MyObj.sub = nh.subscribe<sensor_msgs::PointCloud2>("/input", 10, boundCloudCallback);

    // Create a ROS publisher for the output
    //MyObj.pub = nh.advertise<pcl_pipeline_utils::CompressedPointCloud>("/output", 10);

    // Spin
    ros::spin ();
}