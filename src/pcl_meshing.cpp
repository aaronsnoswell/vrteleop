
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
#include <pcl/surface/poisson.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

#include <pcl/filters/filter.h>

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include "pcl_utils.h"
#include "vrteleop/PolygonMesh.h"
#include "vrteleop/Polygon.h"

#define RAD_TO_DEG  57.29578
#define DEG_TO_RAD  (1.0/RAD_TO_DEG)


/**
 * Simple class to allow meshing of a point cloud
 */
class Meshing
{
public:

    /**
     * Simple Enum to store mesh method
     */
    enum EMeshMethod
    {
        GreedyProjection,
        Poisson,
        MarchingCubesRBF,
        MarchingCubesHoppe,
        ConvexHull
    };


    /**
     * Simple struct to store meshing config
     */
    typedef struct MeshingConfiguration
    {
        EMeshMethod method;

        double greedy_search_radius;
        double greedy_mu;
        int greedy_max_nearerst_neighbours;
        double greedy_max_surface_angle;
        double greedy_min_surface_angle;
        double greedy_max_angle;
        
        int poisson_depth;

        // Constructor
        MeshingConfiguration()
        {
            method = GreedyProjection;

            greedy_search_radius = 2.5;
            greedy_mu = 2.5;
            greedy_max_nearerst_neighbours = 100;
            greedy_max_surface_angle = 45;
            greedy_min_surface_angle = 10;
            greedy_max_angle = 120;

            poisson_depth = 9;
        }

    } MeshingConfiguration_t;


    /**
     * Constructor
     */
    Meshing(MeshingConfiguration_t config)
        :
        _config(config),
        _pclCloud(new pcl::PointCloud<pcl_utils::PointT>),
        _pclCloudNaNsCleared(new pcl::PointCloud<pcl_utils::PointT>),
        _outputMsgPointCloud2(new sensor_msgs::PointCloud2),
        _outputMsg(new vrteleop::PolygonMesh)
    {
        switch(_config.method)
        {
            case GreedyProjection:
            {
                ROS_INFO(
                    "Initializing mesher with mode GreedyProjection"
                );
                break;
            }
            case Poisson:
            {
                ROS_INFO(
                    "Initializing mesher with mode Poisson"
                );
                break;
            }
            default:
            {
                ROS_WARN(
                    "Error: Initialized Mesher with unimplemented mode"
                );
            }
        }
    };


    ros::Subscriber sub;
    ros::Publisher pub;
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

    static MeshingConfiguration_t getConfiguration(ros::NodeHandle nh);

private:
    MeshingConfiguration _config;
    pcl::PointCloud<pcl_utils::PointT>::Ptr _pclCloud;
    pcl::PointCloud<pcl_utils::PointT>::Ptr _pclCloudNaNsCleared;
    sensor_msgs::PointCloud2::Ptr _outputMsgPointCloud2;
    vrteleop::PolygonMesh::Ptr _outputMsg;
};


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

        // Clear NaNs from the input
        std::vector<int> nanIndices;
        pcl::removeNaNFromPointCloud(*_pclCloud, *_pclCloudNaNsCleared, nanIndices);

        // TODO check for INF in input?
        // https://github.com/PointCloudLibrary/pcl/blob/master/examples/common/example_check_if_point_is_valid.cpp
        // Better yet, figure out why MLS is giving us INF

        // Estimate normals
        // NB: We have previously done this, but the compression step discards this info
        pcl::NormalEstimation<pcl_utils::PointT, pcl::PointNormal> n;
        pcl::PointCloud<pcl::PointNormal>::Ptr normalCloud(new pcl::PointCloud<pcl::PointNormal>);
        pcl::search::KdTree<pcl_utils::PointT>::Ptr normalEstimationTree(new pcl::search::KdTree<pcl_utils::PointT>);
        normalEstimationTree->setInputCloud(_pclCloudNaNsCleared);
        n.setInputCloud(_pclCloudNaNsCleared);
        n.setSearchMethod(normalEstimationTree);
        n.setKSearch(20);
        n.compute(*normalCloud);

        // Concatenate the XYZ and normal fields
        // NB: The order you pass the parameters to pcl::concatenateFields matters!
        // If you pass normalCloud second, it's x, y, z fields (that are 0) are copied over to the output!
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
        pcl::concatenateFields(*normalCloud, *_pclCloudNaNsCleared, *combined_cloud);

        // Initialize mesh
        pcl::PolygonMesh mesh;

        // Switch on triangulation method
        switch(_config.method)
        {
            case GreedyProjection:
            {

                // Use Greedy Projection Triangulation
                pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;

                // Create search tree
                pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr meshTree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
                meshTree->setInputCloud(combined_cloud);

                // Set configurable parameters
                gp3.setSearchRadius(_config.greedy_search_radius);
                gp3.setMu(_config.greedy_mu);
                gp3.setMaximumNearestNeighbors(_config.greedy_max_nearerst_neighbours);
                gp3.setMaximumSurfaceAngle(_config.greedy_max_surface_angle * DEG_TO_RAD);
                gp3.setMinimumAngle(_config.greedy_min_surface_angle * DEG_TO_RAD);
                gp3.setMaximumAngle(_config.greedy_max_angle * DEG_TO_RAD);

                // Set hard-coded parameters
                gp3.setNormalConsistency(true);
                gp3.setConsistentVertexOrdering(true);

                // Do the meshing
                gp3.setInputCloud(combined_cloud);
                gp3.setSearchMethod(meshTree);
                gp3.reconstruct(mesh);

                // Additional vertex information
                //std::vector<int> parts = gp3.getPartIDs();
                //std::vector<int> states = gp3.getPointStates();

                break;
            }
            case Poisson:
            {
                pcl::Poisson<pcl::PointXYZRGBNormal> poisson;

                // Set configurable parameters
                poisson.setDepth(_config.poisson_depth);

                // Do the meshing
                poisson.setInputCloud(combined_cloud);
                poisson.reconstruct(mesh);

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

        // Convert to PolygonMesh message format
        pcl::PCLPointCloud2 pcl_pc2_out;
        pcl::toPCLPointCloud2(*combined_cloud, pcl_pc2_out);
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
        vrteleop::Polygon poly;
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
        /*
        ROS_INFO(
            "Meshing: Received input cloud but there are no subscribers - not publishing"
        );
        */
    }
}


// TODO: Figure out better parameters for GP3
// https://github.com/PointCloudLibrary/pcl/blob/master/tools/gp3_surface.cpp
Meshing::MeshingConfiguration_t Meshing::getConfiguration(ros::NodeHandle nh)
{
    MeshingConfiguration_t c;

    std::string method;
    if(nh.getParam("method", method))
    {
        if(method == "GreedyProjection") c.method = GreedyProjection;
        else if(method == "Poisson") c.method = Poisson;
        else if(method == "MarchingCubesRBF") c.method = MarchingCubesRBF;
        else if(method == "MarchingCubesHoppe") c.method = MarchingCubesHoppe;
        else if(method == "ConvexHull") c.method = ConvexHull;
    }

    nh.getParam("greedy_search_radius", c.greedy_search_radius);
    nh.getParam("greedy_mu", c.greedy_mu);
    nh.getParam("greedy_max_nearerst_neighbours", c.greedy_max_nearerst_neighbours);
    nh.getParam("greedy_max_surface_angle", c.greedy_max_surface_angle);
    nh.getParam("greedy_min_surface_angle", c.greedy_min_surface_angle);
    nh.getParam("greedy_max_angle", c.greedy_max_angle);

    nh.getParam("poisson_depth", c.poisson_depth);

    return c;
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
    Meshing MyObj(Meshing::getConfiguration(pnh));
    const boost::function< void(const sensor_msgs::PointCloud2ConstPtr &)> boundCloudCallback = \
        boost::bind(&Meshing::cloudCallback, &MyObj, _1);

    // Create a ROS subscriber for the input
    MyObj.sub = nh.subscribe<sensor_msgs::PointCloud2>("/input", 10, boundCloudCallback);

    // Create a ROS publisher for the output
    MyObj.pub = nh.advertise<vrteleop::PolygonMesh>("/output", 10);

    // Spin
    ros::spin ();
}