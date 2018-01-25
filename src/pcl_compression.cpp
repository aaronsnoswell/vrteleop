
#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>


/**
 * Simple class to compress a point cloud
 * Based on http://pointclouds.org/documentation/tutorials/compression.php#octree-compression
 *
 * TODO Implement this!
 */
class MovingLeastSquares {
private:
    double _search_radius;

public:
    MovingLeastSquares(double search_radius = 0.03)
        : _search_radius(search_radius)
    {
        // Pass
    };

    ros::Subscriber sub;
    ros::Publisher pub;
    void cloudCallback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
};


/**
 * Callback that performs the Point Cloud downsapling
 */
void MovingLeastSquares::cloudCallback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Container for original & filtered data
    pcl::PCLPointCloud2 cloud;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, cloud);

    // Convert to dumbcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr dumb_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    //pcl::MsgFieldMap field_map;
    //pcl::createMapping<pcl::PointXYZ>(cloud_msg->fields, field_map);
    //pcl::fromPCLPointCloud2<pcl::PointXYZ>(cloud, *dumb_cloud);
    pcl::fromPCLPointCloud2<pcl::PointXYZ>(cloud, *dumb_cloud);

    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals (true);

    // Set parameters
    mls.setInputCloud (dumb_cloud);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (_search_radius);

    // Reconstruct
    mls.process (mls_points);

    // Convert from dumbcloud to cloud
    pcl::PCLPointCloud2 cloud_filtered;
    pcl::toPCLPointCloud2(mls_points, cloud_filtered);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(cloud_filtered, output);

    // Publish the data
    pub.publish (output);
}


/**
 * Main
 */
int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pcl_mls");
    ros::NodeHandle nh("~");

    // Read optional leaf_size argument
    double search_radius = 0.03;
    if (nh.hasParam("search_radius"))
    {
        nh.getParam("search_radius", search_radius);
        ROS_INFO("Using %0.4f as search radius", search_radius);
    }

    // Create our filter
    MovingLeastSquares MovingLeastSquaresObj(search_radius);
    const boost::function< void(const sensor_msgs::PointCloud2ConstPtr &)> boundCloudCallback = boost::bind(&MovingLeastSquares::cloudCallback, &MovingLeastSquaresObj, _1);

    // Create a ROS subscriber for the input point cloud
    MovingLeastSquaresObj.sub = nh.subscribe<sensor_msgs::PointCloud2> ("/input", 10, boundCloudCallback);

    // Create a ROS publisher for the output point cloud
    MovingLeastSquaresObj.pub = nh.advertise<sensor_msgs::PointCloud2> ("/output", 10);

    // Spin
    ros::spin ();
}