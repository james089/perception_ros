#ifndef POINT_CLOUD_REGISTER_H
#define POINT_CLOUD_REGISTER_H

#endif // POINT_CLOUD_REGISTER_H


#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <ros/ros.h>

namespace perception
{
    using pcl::visualization::PointCloudColorHandlerGenericField;
    using pcl::visualization::PointCloudColorHandlerCustom;

    //convenient typedefs
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef pcl::PointNormal PointNormalT;
    typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

    //convenient structure to handle our pointclouds
    struct PCD
    {
      PointCloud::Ptr cloud;
      std::string f_name;

      PCD() : cloud (new PointCloud) {};
    };

    struct PCDComparator
    {
      bool operator () (const PCD& p1, const PCD& p2)
      {
        return (p1.f_name < p2.f_name);
      }
    };

    // Define a new point representation for < x, y, z, curvature >
    class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
    {
      using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
    public:
      MyPointRepresentation ()
      {
        // Define the number of dimensions
        nr_dimensions_ = 4;
      }

      // Override the copyToFloatArray method to define our feature vector
      virtual void copyToFloatArray (const PointNormalT &p, float * out) const
      {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
      }
    };


    struct Config
    {
        /** \brief This is root folder where we load and save data
          */
        std::string data_path = "/home/mostafa/staubli_vision_ws/src/perception/data";

        /** \brief Downsample points, filter size
          */
        float voxel_grid_leaf_size = 0.05;

        /** \brief Paramter for normal estimation k search
          */
        int norm_est_k_search_val = 30;

        /** \brief Set the maximum distance between two correspondences (src<->tgt) to a value (m)
         *  Note: adjust this based on the size of your datasets
          */
        float max_corresp_size_init = 0.01;

        /** \brief if the difference between this transformation and the previous one
         * is smaller than the threshold, refine the process by reducing the maximal correspondence distance
          */
        float max_corresp_size_drop_step = 0.001;

        /** \brief Total interations
          */
        float total_iterations = 30;
    };


    class PointCloudRegister
    {
    public:
        PointCloudRegister();
        virtual ~PointCloudRegister();

        // Functions
        void LoadParam();
        void RunRegister(int argc, char **argv);

        // Values
        Config config;
    protected:
        void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source);
        void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source);
        void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models);
        void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample);

    protected:
        pcl::visualization::PCLVisualizer *p;
        //its left and right viewports
        int vp_1, vp_2;
    };
}
