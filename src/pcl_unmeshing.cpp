
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/gp3.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include "pcl_utils.h"
#include "vrteleop/PolygonMesh.h"
#include "vrteleop/Polygon.h"


/**
 * Simple class to allow meshing of a point cloud
 */
class UnMeshing
{
public:

    UnMeshing()
        :
        _pclCloud(new pcl::PointCloud<pcl_utils::PointT>),
        _outputMsg(new sensor_msgs::PointCloud2)
    {
        // Pass
    };


    ros::Subscriber sub;
    ros::Publisher pub;
    void cloudCallback(const vrteleop::PolygonMeshPtr& msg);

private:
    pcl::PointCloud<pcl_utils::PointT>::Ptr _pclCloud;
    sensor_msgs::PointCloud2::Ptr _outputMsg;
};


/**
 * Callback that performs the Point Cloud un-meshing
 */
void UnMeshing::cloudCallback(const vrteleop::PolygonMeshPtr& msg)
{
    if(pub.getNumSubscribers())
    {
        // Convert from sensor_msg::PointCloud2 to pcl::PointCloud<pcl_utils::PointT>
        //pcl::PCLPointCloud2 pcl_pc2;
        //pcl_conversions::toPCL(*msg, pcl_pc2);
        //pcl::fromPCLPointCloud2(pcl_pc2, *_pclCloud);

        // Convert to PolygonMesh message format
        //pcl::PCLPointCloud2 pcl_pc2;
        //pcl::toPCLPointCloud2(*normalCloud, pcl_pc2);
        //pcl_conversions::fromPCL(pcl_pc2, *_outputMsgPointCloud2);

        // Copy header
        _outputMsg->header = msg->header;

        // Copy over cloud
        _outputMsg->height = msg->cloud.height;
        _outputMsg->width = msg->cloud.width;
        _outputMsg->fields = msg->cloud.fields;
        _outputMsg->is_bigendian = msg->cloud.is_bigendian;
        _outputMsg->point_step = msg->cloud.point_step;
        _outputMsg->row_step = msg->cloud.row_step;
        _outputMsg->data = msg->cloud.data;
        _outputMsg->is_dense = msg->cloud.is_dense;

        // Pubish the mesh
        pub.publish(_outputMsg);
    }
    else
    {
       ROS_INFO(
           "UnMeshing: Received input cloud but there are no subscribers - not publishing"
       );
    }
}


/**
 * Main
 */
int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pcl_unmeshing");
    ros::NodeHandle nh, pnh("~");

    // Create our filter
    UnMeshing MyObj;
    const boost::function< void(const vrteleop::PolygonMeshPtr &)> boundCloudCallback = \
        boost::bind(&UnMeshing::cloudCallback, &MyObj, _1);

    // Create a ROS subscriber for the input
    MyObj.sub = nh.subscribe<vrteleop::PolygonMesh>("/input", 10, boundCloudCallback);

    // Create a ROS publisher for the output
    MyObj.pub = nh.advertise<sensor_msgs::PointCloud2>("/output", 10);

    // Spin
    ros::spin ();
}