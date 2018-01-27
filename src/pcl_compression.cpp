
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
//#include "compressedpointcloud.h"
#include "ros_msg_convert.h"


typedef pcl::PointXYZRGB PointT;


/**
 * Simple class to allow copmression of a point cloud
 * Originally from https://github.com/ChenxiTU/OctreeCompression4Ros
 */
class Compression {
private:
    
    pcl_pipeline_utils::CompressedPointCloud _outputMsg;
    pcl::io::OctreePointCloudCompression<PointT>* _PointCloudEncoder;
    pcl::PointCloud<PointT>::Ptr _pclCloud;

public:
    Compression(pcl::io::OctreePointCloudCompression<PointT>* PointCloudEncoder)
        :
        _PointCloudEncoder(PointCloudEncoder),
        _pclCloud(new pcl::PointCloud<PointT>)
    {
        // Pass
    };


    ros::Subscriber sub;
    ros::Publisher pub;
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
};


/**
 * Callback that performs the Point Cloud downsapling
 */
void Compression::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if(pub.getNumSubscribers())
    {
        // Stringstream to store compressed point cloud
        std::stringstream compressedData;

        // Must convert from sensor_msg::PointCloud2 to pcl::PointCloud<PointT> for the encoder
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::fromPCLPointCloud2 (pcl_pc2, *_pclCloud);

        _PointCloudEncoder->encodePointCloud(_pclCloud, compressedData);

        _outputMsg.header = msg->header;
        _outputMsg.data = compressedData.str();
        pub.publish(_outputMsg);

        //long original_size = sizeof(msg->data);
        //int compressed_size = sizeof(_outputMsg);
        //ROS_INFO(
        //    "Published cloud, original size %ld bytes, compressed size %d bytes, %03.2f%% of original.",
        //    original_size,
        //    compressed_size,
        //    (float)compressed_size/(float)original_size*100
        //);
    }
    else
    {
        ROS_INFO(
            "Received input cloud but there are no subscribers - not publishing"
        );
    }
}

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
 * Read a Compression Profile String from a node handle
 */
pcl::io::compression_Profiles_e getCompressionProfile(ros::NodeHandle nh, ConfigurationProfile *c = 0)
{
    std::string compression_profile;
    if(nh.getParam("profile", compression_profile))
    {
        if(compression_profile == "LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR")        return pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
        else if(compression_profile == "LOW_RES_ONLINE_COMPRESSION_WITH_COLOR")      return pcl::io::LOW_RES_ONLINE_COMPRESSION_WITH_COLOR;
        else if(compression_profile == "MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR")   return pcl::io::MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
        else if(compression_profile == "MED_RES_ONLINE_COMPRESSION_WITH_COLOR")      return pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
        else if(compression_profile == "HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR")  return pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
        else if(compression_profile == "HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR")     return pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR;
        else if(compression_profile == "LOW_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR")  return pcl::io::LOW_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR;
        else if(compression_profile == "LOW_RES_OFFLINE_COMPRESSION_WITH_COLOR")     return pcl::io::LOW_RES_OFFLINE_COMPRESSION_WITH_COLOR;
        else if(compression_profile == "HIGH_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR") return pcl::io::HIGH_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR;
        else if(compression_profile == "HIGH_RES_OFFLINE_COMPRESSION_WITH_COLOR")    return pcl::io::HIGH_RES_OFFLINE_COMPRESSION_WITH_COLOR;
        else return pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
    }
    else
    {   
        if(c != 0)
        {   
            if(nh.hasParam("showStatistics"))
            {
                nh.getParam("showStatistics", c->showStatistics);
            }
            
            if(nh.hasParam("pointResolution"))
            {
                nh.getParam("pointResolution", c->pointResolution);
            }

            if(nh.hasParam("octreeResolution"))
            {
                nh.getParam("octreeResolution", c->octreeResolution);
            }

            if(nh.hasParam("doVoxelGridDownDownSampling"))
            {
                nh.getParam("doVoxelGridDownDownSampling", c->doVoxelGridDownDownSampling);
            }

            if(nh.hasParam("iFrameRate"))
            {
                nh.getParam("iFrameRate", (int &)(c->iFrameRate));
            }

            if(nh.hasParam("doColorEncoding"))
            {
                nh.getParam("doColorEncoding", c->doColorEncoding);
            }

            if(nh.hasParam("colorBitResolution"))
            {
                nh.getParam("colorBitResolution", (int &)(c->colorBitResolution));
            }
        }

        return pcl::io::MANUAL_CONFIGURATION;
    }
}


/**
 * Main
 */
int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pcl_compression");
    ros::NodeHandle nh, pnh("~");

    ConfigurationProfile profile;
    pcl::io::compression_Profiles_e compressionProfile = getCompressionProfile(pnh, &profile);

    // Create our filter
    Compression CompressionObj(
        new pcl::io::OctreePointCloudCompression<PointT>(
            compressionProfile,
            profile.showStatistics,
            profile.pointResolution,
            profile.octreeResolution,
            profile.doVoxelGridDownDownSampling,
            profile.iFrameRate,
            profile.doColorEncoding,
            static_cast<unsigned char> (profile.colorBitResolution)
        )
    );
    const boost::function< void(const sensor_msgs::PointCloud2ConstPtr &)> boundCloudCallback = \
        boost::bind(&Compression::cloudCallback, &CompressionObj, _1);

    // Create a ROS subscriber for the input point cloud
    CompressionObj.sub = nh.subscribe<sensor_msgs::PointCloud2> ("/input", 10, boundCloudCallback);

    // Create a ROS publisher for the output point cloud
    CompressionObj.pub = nh.advertise<pcl_pipeline_utils::CompressedPointCloud> ("/output", 10);

    // Spin
    ros::spin ();
}