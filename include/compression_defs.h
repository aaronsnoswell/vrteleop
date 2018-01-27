

// Typedef for the kind of points we can compress
// NB: PointXYZRGBNormal doesn't seem to be supported by pcl/compression/octree_pointcloud_compression
typedef pcl::PointXYZRGB PointT;


/**
 * Simple struct to store compression profile
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


