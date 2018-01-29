
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
        _pclCloud(new pcl::PointCloud<pcl::PointXYZ>),
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr _pclCloud;
    sensor_msgs::PointCloud2::Ptr _outputMsgPointCloud2;
    pcl_pipeline_utils::PolygonMesh::Ptr _outputMsg;
};


/**
 * Copied from http://docs.pointclouds.org/1.0.1/io_8hpp_source.html#l00187
 */
template <typename PointIn1T, typename PointIn2T, typename PointOutT> void
foozleConcatenateFields (const pcl::PointCloud<PointIn1T> &cloud1_in,
                        const pcl::PointCloud<PointIn2T> &cloud2_in,
                        pcl::PointCloud<PointOutT> &cloud_out)
{
    typedef typename pcl::traits::fieldList<PointIn1T>::type FieldList1;
    typedef typename pcl::traits::fieldList<PointIn2T>::type FieldList2;

    ROS_WARN("Starting concat");

    if (cloud1_in.points.size () != cloud2_in.points.size ())
    {
        PCL_ERROR ("[pcl::concatenateFields] The number of points in the two input datasets differs!\n");
        return;
    }

    ROS_WARN("Same # of points");

    // Resize the output dataset
    cloud_out.points.resize(cloud1_in.points.size());

    ROS_WARN("Resized");

    cloud_out.header   = cloud1_in.header;
    cloud_out.width    = cloud1_in.width;
    cloud_out.height   = cloud1_in.height;

    if (!cloud1_in.is_dense || !cloud2_in.is_dense)
        cloud_out.is_dense = false;
    else
        cloud_out.is_dense = true;

    ROS_WARN("Copied metadata");

    // Iterate over each point
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
    {
        // Iterate over each dimension
        pcl::for_each_type <FieldList1> (pcl::NdConcatenateFunctor <PointIn1T, PointOutT> (cloud1_in.points[i], cloud_out.points[i]));
        pcl::for_each_type <FieldList2> (pcl::NdConcatenateFunctor <PointIn2T, PointOutT> (cloud2_in.points[i], cloud_out.points[i]));
    }

    ROS_WARN("Copied points");
}


/**
 * Callback that performs the Point Cloud meshing
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
        pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> n;
        pcl::PointCloud<pcl::PointNormal>::Ptr normalCloud(new pcl::PointCloud<pcl::PointNormal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr normalEstimationTree(new pcl::search::KdTree<pcl::PointXYZ>);
        normalEstimationTree->setInputCloud(_pclCloud);
        n.setInputCloud(_pclCloud);
        n.setSearchMethod(normalEstimationTree);
        n.setKSearch(20);
        n.compute(*normalCloud);

        // Concatenate the XYZ and normal fields
        // NB: The order you pass the parameters to pcl::concatenateFields matters!
        // If you pass normalCloud second, it's x, y, z fields (that are 0) are copied over to the output!
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
        pcl::concatenateFields(*normalCloud, *_pclCloud, *cloud_with_normals);

        // Initialize mesh
        pcl::PolygonMesh mesh;

        // Switch on triangulation method
        switch(_method)
        {
            case GreedyProjection:
            {

                // Use Greedy Projection Triangulation
                pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

                // Create search tree
                pcl::search::KdTree<pcl::PointNormal>::Ptr meshTree(new pcl::search::KdTree<pcl::PointNormal>);
                meshTree->setInputCloud(cloud_with_normals);

                // Set the maximum distance between connected points (maximum edge length)
                gp3.setSearchRadius(0.025);

                // Set typical values for the parameters
                gp3.setMu(2.5);
                gp3.setMaximumNearestNeighbors(100);
                gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
                gp3.setMinimumAngle(M_PI/18); // 10 degrees
                gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
                gp3.setNormalConsistency(true);
                gp3.setConsistentVertexOrdering(true);

                // Get result
                gp3.setInputCloud(cloud_with_normals);
                gp3.setSearchMethod(meshTree);
                gp3.reconstruct(mesh);

                // Additional vertex information
                std::vector<int> parts = gp3.getPartIDs();
                std::vector<int> states = gp3.getPointStates();

                break;
            }
            case Poisson:
            {
                ROS_WARN(
                    "Poisson meshing method not implemented yet"
                );

                //Poisson<pcl::PointNormal> poisson;
                //poisson.setDepth (9);
                //poisson.setInputCloud (cloud_with_normals);
                //poisson.reconstruct(mesh);

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
        
        // Save output file
        // (For debugging only)
        pcl::io::saveVTKFile("/home/aaron/Development/mesh.vtk", mesh);
        //pcl::io::savePolygonFileSTL("/home/aaron/Development/testing-output.stl", mesh);
        ros::shutdown();

        // Convert to PolygonMesh message format
        pcl::PCLPointCloud2 pcl_pc2_out;
        pcl::toPCLPointCloud2(*cloud_with_normals, pcl_pc2_out);
        pcl_conversions::fromPCL(pcl_pc2_out, *_outputMsgPointCloud2);

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

        // Copy over the mesh array
        pcl_pipeline_utils::Polygon poly;
        for(int i=0; i<mesh.polygons.size(); i++)
        {
            poly.vertices.clear();
            for(int j=0; j<mesh.polygons[i].vertices.size(); j++)
            {
                poly.vertices.push_back(mesh.polygons[i].vertices[j]);
            }
            _outputMsg->polygons.push_back(poly);
        }

        // Pubish the mesh
        pub.publish(_outputMsg);

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