
#include "ros_msg_convert.h"


pcl::PointCloud<pcl::PointXYZ>::Ptr ros_msg_convert::fromROSMsg(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
    return cloud;   
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr ros_msg_convert::fromROSMsgRGB(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
    return cloud;   
}


sensor_msgs::PointCloud2::Ptr ros_msg_convert::toROSMsg(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(cloud, *msg);
    return msg; 
}


sensor_msgs::PointCloud2::Ptr ros_msg_convert::toROSMsg(pcl::PointCloud<pcl::PointNormal>& cloud)
{
    sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(cloud, *msg);
    return msg; 
}


sensor_msgs::PointCloud2::Ptr ros_msg_convert::toROSMsg(pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud)
{
    sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(cloud, *msg);
    return msg; 
}
