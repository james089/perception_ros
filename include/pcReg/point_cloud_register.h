#ifndef POINT_CLOUD_REGISTER_H
#define POINT_CLOUD_REGISTER_H

#endif // POINT_CLOUD_REGISTER_H

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/common/time.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <ros/ros.h>

namespace pcReg
{
    using pcl::visualization::PointCloudColorHandlerGenericField;
    using pcl::visualization::PointCloudColorHandlerCustom;

    // convenient typedefs
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef pcl::PointNormal PointNT;
    typedef pcl::PointCloud<PointNT> PointCloudNT;    //point cloud with normals


    // Type of align algorithms we use
    enum Algorithm
    {
      icp,
      prerejective,
      fpcs,
    };
    const char* ToString(Algorithm v);

    // convenient structure to handle our pointclouds
    struct PCD
    {
      PointCloudT::Ptr cloud;
      std::string f_name;

      PCD() : cloud (new PointCloudT) {};
    };

    struct PCDComparator
    {
      bool operator () (const PCD& p1, const PCD& p2)
      {
        return (p1.f_name < p2.f_name);
      }
    };

    // Define a new point representation for < x, y, z, curvature >
    class MyPointRepresentation : public pcl::PointRepresentation <PointNT>
    {
      using pcl::PointRepresentation<PointNT>::nr_dimensions_;
    public:
      MyPointRepresentation ()
      {
        // Define the number of dimensions
        nr_dimensions_ = 4;
      }

      // Override the copyToFloatArray method to define our feature vector
      virtual void copyToFloatArray (const PointNT &p, float * out) const
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
        /* This is root folder where we load and save data */
        std::string data_path = "/home/mostafa/staubli_vision_ws/src/perception/data";

        /* Type of search algorithm */
        int type_reg_algorithm = 0;

        /* Is reduce noise */
        bool is_reduce_noise = false;

        /* Is duplicate source cloud */
        bool is_duplicate_source = false;

        // ======================== Correspondence setting =======================
        /* Downsample points, filter size */
        double voxel_grid_leaf_size = 0.05;

        // ======================== Norm estimation setting=======================
        /* Paramter for normal estimation k search */
        int norm_est_k_search_val = 30;

        // ======================== Correspondence setting =======================
        /* Set the maximum distance between two correspondences (src<->tgt) to a value (m)
         * Note: adjust this based on the size of your datasets */
        double max_corresp_size_init = 0.01;

        /* The epsilon (difference) between the previous transformation 
         * and the current estimated transformation threshold*/
        double trans_epsilon = 1e-6;

        /* the iteration for each align*/
        int max_sub_iteration = 2;

        /* euclideanFitnessEpsilon */
        bool is_set_eucl_fit_epsl = true;

        /* euclideanFitnessEpsilon */
        double euclidean_fitness_epsilon = 1;

        /* Total interations */
        int total_iterations = 30;

        /* Feature estimation, radius */
        double feat_radius = 1;
        // ================= For Prerejective ============================

        /* Number of points to sample for generating/prerejecting a pose */
        int num_samples = 3;

        /* Number of nearest features to use */
        int corresp_randomness = 5;

        /* Polygonal edge length similarity threshold */
        float similarity_thresh = 0.9f;

    };


    class PointCloudRegister
    {
      public:
          PointCloudRegister();
          virtual ~PointCloudRegister();

          // Functions
          void LoadParam();
          void RunRegister(int argc, char **argv, Algorithm alg);

          // Values
          Config config;
      protected:
          void showCloudsLeft(const PointCloudT::Ptr cloud_target, const PointCloudT::Ptr cloud_source);
          void showCloudsRight(const PointCloudNT::Ptr cloud_target, const PointCloudNT::Ptr cloud_source);
          void loadStl (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models);
          void loadPCD (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models);
          void pairAlign_icp (const PointCloudT::Ptr cloud_src, const PointCloudT::Ptr cloud_tgt, PointCloudT::Ptr output, Eigen::Matrix4f &final_transform, bool downsample);
          void pairAlign_prerej (const PointCloudT::Ptr cloud_src, const PointCloudT::Ptr cloud_tgt, PointCloudT::Ptr output, Eigen::Matrix4f &final_transform, bool downsample);

      protected:
          pcl::visualization::PCLVisualizer *p;
          //its left and right viewports
          int vp_1, vp_2;
    };
}
