
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
#include "pcl_pipeline_utils/PolygonMesh.h"
#include "pcl_pipeline_utils/Polygon.h"


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
        _pclCloud(new pcl::PointCloud<pcl_utils::PointT>),
        _outputMsgPointCloud2(new sensor_msgs::PointCloud2),
        _outputMsg(new pcl_pipeline_utils::PolygonMesh)
    {
        // Pass
    };


    ros::Subscriber sub;
    ros::Publisher pub;
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

private:
    EMeshMethod _method;
    pcl::PointCloud<pcl_utils::PointT>::Ptr _pclCloud;
    sensor_msgs::PointCloud2::Ptr _outputMsgPointCloud2;
    pcl_pipeline_utils::PolygonMesh::Ptr _outputMsg;
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

        // Estimate normals
        // NB: We have previously done this, but the compression step discards this info
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> n;
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normalCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr normalEstimationTree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        normalEstimationTree->setInputCloud(_pclCloud);
        n.setInputCloud(_pclCloud);
        n.setSearchMethod(normalEstimationTree);
        n.setKSearch(20);
        n.compute(*normalCloud);
        n.setViewPoint(0, 0, 0);

        // Initialize mesh
        pcl::PolygonMesh triangles;

        switch(_method)
        {
            case GreedyProjection:
            {

                // Use Greedy Projection Triangulation
                pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;

                // Create search tree
                pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr meshTree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
                meshTree->setInputCloud(normalCloud);

                // Set the maximum distance between connected points (maximum edge length)
                gp3.setSearchRadius (25);

                // Set typical values for the parameters
                gp3.setMu (2.5);
                gp3.setMaximumNearestNeighbors (100);
                gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
                gp3.setMinimumAngle(M_PI/18); // 10 degrees
                gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
                gp3.setNormalConsistency(true);
                gp3.setConsistentVertexOrdering(true);

                // Get result
                gp3.setInputCloud(normalCloud);
                gp3.setSearchMethod(meshTree);
                gp3.reconstruct(triangles);

                // Additional vertex information
                std::vector<int> parts = gp3.getPartIDs();
                std::vector<int> states = gp3.getPointStates();

                // Convert to PolygonMesh message format
                pcl::PCLPointCloud2 pcl_pc2;
                pcl::toPCLPointCloud2(*normalCloud, pcl_pc2);
                pcl_conversions::fromPCL(pcl_pc2, *_outputMsgPointCloud2);

                // Copy header
                _outputMsg->header = msg->header;

                // Copy over cloud
                _outputMsg->cloud.height = _outputMsgPointCloud2->height;
                _outputMsg->cloud.width = _outputMsgPointCloud2->width;
                _outputMsg->cloud.fields = _outputMsgPointCloud2->fields;
                _outputMsg->cloud.is_bigendian = _outputMsgPointCloud2->is_bigendian;
                _outputMsg->cloud.point_step = _outputMsgPointCloud2->point_step;
                _outputMsg->cloud.row_step = _outputMsgPointCloud2->row_step;
                _outputMsg->cloud.data = _outputMsgPointCloud2->data;
                _outputMsg->cloud.is_dense = _outputMsgPointCloud2->is_dense;

                // Copy over the triangles array
                pcl_pipeline_utils::Polygon poly;
                for(int i=0; i<triangles.polygons.size(); i++)
                {
                    poly.vertices.clear();
                    for(int j=0; j<triangles.polygons[i].vertices.size(); j++)
                    {
                        poly.vertices.push_back(triangles.polygons[i].vertices[j]);
                    }
                    _outputMsg->polygons.push_back(poly);
                }
                
                // Save output file
                // (Debug only)
                //pcl::io::saveVTKFile("/home/aaron/Development/testing-output.vtk", triangles);
                //pcl::io::savePolygonFileSTL("/home/aaron/Development/testing-output.stl", triangles);
                //ros::shutdown();

                // Pubish the mesh
                pub.publish(_outputMsg);

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
           "Meshing: Received input cloud but there are no subscribers - not publishing"
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
    MyObj.pub = nh.advertise<pcl_pipeline_utils::PolygonMesh>("/output", 10);

    // Spin
    ros::spin ();
}