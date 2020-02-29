#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

using mCloudT = pcl::PointCloud<pcl::PointXYZ>;
const std::string DATA_PATH = "/home/mostafa/staubli_vision_ws/src/perception/data";

void viewOnce (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (0, 0, 0);
    pcl::PointXYZ o;
    o.x = 0.0;
    o.y = 0;
    o.z = 0;
    //viewer.addSphere (o, 0.25, "sphere", 0);
    //std::cout << "i only run once" << std::endl;

}

void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);
}

// ===============================================================================================



// ===============================================================================================

int main(int argc, char *argv[])
{
    // cloud smart pointer
    mCloudT::Ptr ptr_cloud1(new mCloudT());
    mCloudT::Ptr ptr_cloud2(new mCloudT());

    // processed point cloud
    mCloudT::Ptr ptr_cloud_prcd1 (new mCloudT());
    mCloudT::Ptr ptr_cloud_prcd2 (new mCloudT());

    // registered point cloud
    mCloudT::Ptr ptr_cloud_reg (new mCloudT());

    // Load PC
    pcl::io::loadPCDFile (DATA_PATH + "/crown_scan1.pcd", *ptr_cloud1);
    pcl::io::loadPCDFile (DATA_PATH + "/crown_scan2.pcd", *ptr_cloud2);

    // Noise reduction
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.setInputCloud (ptr_cloud1);
    sor.filter (*ptr_cloud_prcd1);
    sor.setInputCloud (ptr_cloud2);
    sor.filter (*ptr_cloud_prcd2);

    // Display
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    //pcl::visualization::PCLVisualizer viewer("Viewer");

    //viewer.addPointCloud(ptr_cloud1, "scan1");
    //viewer.addPointCloud(ptr_cloud2, "scan2");

    //blocks until the cloud is actually rendered
    viewer.showCloud(ptr_cloud1, "scan1");
    viewer.showCloud(ptr_cloud2, "scan2");

    //This will only get called once
    //viewer.runOnVisualizationThreadOnce (viewOnce);
    //This will get called once per visualization iteration
    //viewer.runOnVisualizationThread (viewerPsycho);

    while (!viewer.wasStopped ())
    {
    }

    return 0;
}
