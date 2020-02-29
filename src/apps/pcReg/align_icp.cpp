#include <pcReg/point_cloud_register.h>
#include <mUtils/mFunctions.h>

namespace pcReg
{
    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Align a pair of PointCloud datasets and return the result
      * \param cloud_src the source PointCloud
      * \param cloud_tgt the target PointCloud
      * \param output the resultant aligned source PointCloud
      * \param final_transform the resultant transform between source and target
      */
    void PointCloudRegister::pairAlign_icp (const PointCloudT::Ptr cloud_src, const PointCloudT::Ptr cloud_tgt, PointCloudT::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
    {
      //
      // Downsample for consistency and speed
      // \note enable this for large datasets
      PointCloudT::Ptr src (new PointCloudT);
      PointCloudT::Ptr tgt (new PointCloudT);
      pcl::VoxelGrid<PointT> grid;
      if (downsample)
      {
        grid.setLeafSize (config.voxel_grid_leaf_size, config.voxel_grid_leaf_size, config.voxel_grid_leaf_size);
        grid.setInputCloud (cloud_src);
        grid.filter (*src);

        grid.setInputCloud (cloud_tgt);
        grid.filter (*tgt);
      }
      else
      {
        src = cloud_src;
        tgt = cloud_tgt;
      }


      // Compute surface normals and curvature
      PointCloudNT::Ptr points_with_normals_src (new PointCloudNT);
      PointCloudNT::Ptr points_with_normals_tgt (new PointCloudNT);

      pcl::NormalEstimation<PointT, PointNT> norm_est;
      pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
      norm_est.setSearchMethod (tree);
      norm_est.setKSearch (config.norm_est_k_search_val);

      norm_est.setInputCloud (src);
      norm_est.compute (*points_with_normals_src);
      pcl::copyPointCloud (*src, *points_with_normals_src);

      norm_est.setInputCloud (tgt);
      norm_est.compute (*points_with_normals_tgt);
      pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

      //
      // Instantiate our custom point representation (defined above) ...
      MyPointRepresentation point_representation;
      // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
      float alpha[4] = {1.0, 1.0, 1.0, 1.0};
      point_representation.setRescaleValues (alpha);

      //
      // Align
      pcl::IterativeClosestPointNonLinear<PointNT, PointNT> reg;
      reg.setTransformationEpsilon (config.trans_epsilon);
      // Set the maximum distance between two correspondences (src<->tgt) to 10cm
      // Note: adjust this based on the size of your datasets
      reg.setMaxCorrespondenceDistance (config.max_corresp_size_init);
      // Set the point representation
      reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

      reg.setInputSource (points_with_normals_src);
      reg.setInputTarget (points_with_normals_tgt);

      //
      // Run the same optimization in a loop and visualize the results
      Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
      PointCloudNT::Ptr reg_result = points_with_normals_src;
      reg.setMaximumIterations (config.max_sub_iteration);
      if(config.is_set_eucl_fit_epsl) reg.setEuclideanFitnessEpsilon (config.euclidean_fitness_epsilon);

      float last_fitness = VTK_FLOAT_MAX;
      //bool is_converged = false;

      for (int i = 0; i < config.total_iterations; ++i)
      {
        PCL_INFO ("Iteration Nr. %d.\n", i);

        // save cloud for visualization purpose
        points_with_normals_src = reg_result;

        // Estimate
        reg.setInputSource (points_with_normals_src);
        reg.align (*reg_result);

        float current_fitness = reg.getFitnessScore();
        //is_converged = reg.hasConverged();

        PCL_INFO ("Fitness score: %f\n", current_fitness);
        //PCL_INFO ("Is converged: %s\n", is_converged? "true" : "false");

        //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation () * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        double current_trans_diff = std::abs ((reg.getLastIncrementalTransformation () - prev).sum ());
        PCL_INFO ("Current trans difference: %f\n", current_trans_diff);

        if (current_trans_diff < reg.getTransformationEpsilon ())
        {
          double newMaxCorrDis = mUtils::drop_even(reg.getMaxCorrespondenceDistance (), config.max_corresp_size_init, config.total_iterations);
          //float newMaxCorrDis = reg.getMaxCorrespondenceDistance () - config.max_corresp_size_init / config.total_iterations;
          //float newMaxCorrDis = 1.0f / ((i + 1) + 1.0f / config.max_corresp_size_init);
          if(newMaxCorrDis >= 0)
          {
            reg.setMaxCorrespondenceDistance (newMaxCorrDis);
            PCL_INFO ("New corres. distance: %f\n", newMaxCorrDis);
          }
        }

        // // Keep reducing fitness epsilon 
        // if(reg.getEuclideanFitnessEpsilon () - config.euclidean_fitness_epsilon / config.total_iterations >= 0)
        // {
        //   float newEuFitEpsl = reg.getEuclideanFitnessEpsilon () - config.euclidean_fitness_epsilon / config.total_iterations;
        //   reg.setEuclideanFitnessEpsilon (newEuFitEpsl);
        //   PCL_INFO ("New Euc Fit Epsl: %f\n", newEuFitEpsl);
        // }
          
        prev = reg.getLastIncrementalTransformation ();

        // visualize current state
        showCloudsRight(points_with_normals_tgt, points_with_normals_src);
      }

      
      // Get the transformation from target to source
      targetToSource = Ti.inverse();

      //
      // Transform target back in source frame
      pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

      p->removePointCloud ("source");
      p->removePointCloud ("target");

      PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
      PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
      p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
      p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);

      PCL_INFO ("Press q to continue the registration.\n");
      p->spin ();

      p->removePointCloud ("source");
      p->removePointCloud ("target");

      //add the source to the transformed target
      *output += *cloud_src;

      final_transform = targetToSource;
     }
}