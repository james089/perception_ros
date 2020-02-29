#include <pcReg/point_cloud_register.h>
#include <mUtils/mFunctions.h>

#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>

namespace pcReg
{
  // Types
  typedef pcl::FPFHSignature33 FeatureT;
  typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
  typedef pcl::PointCloud<FeatureT> FeatureCloudT;
  typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

  const std::string DATA_PATH = "/home/mostafa/staubli_vision_ws/src/perception/data";

  void PointCloudRegister::pairAlign_prerej (const PointCloudT::Ptr cloud_src, const PointCloudT::Ptr cloud_tgt, PointCloudT::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
  {
    PointCloudT::Ptr src (new PointCloudT);
    PointCloudT::Ptr tgt (new PointCloudT);

    PointCloudNT::Ptr object (new PointCloudNT);
    PointCloudNT::Ptr object_aligned (new PointCloudNT);
    PointCloudNT::Ptr scene (new PointCloudNT);
    FeatureCloudT::Ptr object_features (new FeatureCloudT);
    FeatureCloudT::Ptr scene_features (new FeatureCloudT);

    // Downsample
    pcl::console::print_highlight ("Downsampling...\n");
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

    // // Estimate normals
    pcl::console::print_highlight ("Estimating scene normals...\n");
    pcl::NormalEstimation<PointT, PointNT> norm_est;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    norm_est.setSearchMethod (tree);
    norm_est.setKSearch (config.norm_est_k_search_val);

    norm_est.setInputCloud (src);
    norm_est.compute (*object);
    pcl::copyPointCloud (*src, *object);

    norm_est.setInputCloud (tgt);
    norm_est.compute (*scene);
    pcl::copyPointCloud (*tgt, *scene);

    // //
    // // Instantiate our custom point representation (defined above) ...
    // MyPointRepresentation point_representation;
    // // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
    // float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    // point_representation.setRescaleValues (alpha);

    // pcl::console::print_highlight ("Estimating scene normals...\n");
    // pcl::NormalEstimationOMP<PointNT,PointNT> nest;
    // nest.setRadiusSearch (1);
    // nest.setInputCloud (scene);
    // nest.compute (*scene);
    
    // Estimate features
    pcl::console::print_highlight ("Estimating features...\n");
    FeatureEstimationT fest;
    fest.setRadiusSearch (config.feat_radius);
    fest.setInputCloud (object);
    fest.setInputNormals (object);
    fest.compute (*object_features);
    fest.setInputCloud (scene);
    fest.setInputNormals (scene);
    fest.compute (*scene_features);
    
    // Perform alignment
    pcl::console::print_highlight ("Starting alignment...\n");
    pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;

    // pcReg::MyPointRepresentation point_representation;
    // // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
    // float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    // point_representation.setRescaleValues (alpha);
    // align.setPointRepresentation (boost::make_shared<const pcReg::MyPointRepresentation> (point_representation));
    
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
    PointCloudNT::Ptr reg_result = object;

    align.setInputSource (object);
    align.setSourceFeatures (object_features);
    align.setInputTarget (scene);
    align.setTargetFeatures (scene_features);
    align.setMaximumIterations (config.max_sub_iteration); // Number of RANSAC iterations
    align.setNumberOfSamples (config.num_samples); // Number of points to sample for generating/prerejecting a pose
    align.setCorrespondenceRandomness (config.corresp_randomness); // Number of nearest features to use
    align.setSimilarityThreshold (config.similarity_thresh); // Polygonal edge length similarity threshold
    align.setMaxCorrespondenceDistance (config.max_corresp_size_init); // Inlier threshold

    for (int i = 0; i < config.total_iterations; ++i)
    {
      PCL_INFO ("Iteration Nr. %d.\n", i);

      // save cloud for visualization purpose
      object = reg_result;

      // Estimate
      align.setInputSource (object);
      align.align (*reg_result);

      align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
      {
        pcl::ScopeTime t("Alignment");
        align.align (*reg_result); //object_aligned);
      }
      
      if (align.hasConverged ())
      {
        // Print results
        // printf ("\n");
        // Eigen::Matrix4f transformation = align.getFinalTransformation ();
        // pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
        // pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
        // pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
        // pcl::console::print_info ("\n");
        // pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
        // pcl::console::print_info ("\n");
        // pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());
        
        Ti = align.getFinalTransformation () * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        double current_trans_diff = std::abs ((align.getLastIncrementalTransformation () - prev).sum ());
        PCL_INFO ("Current trans difference: %f\n", current_trans_diff);

        if (current_trans_diff < align.getTransformationEpsilon ())
        {
          //float newMaxCorrDis = align.getMaxCorrespondenceDistance () - config.max_corresp_size_init / config.total_iterations;
          //float newMaxCorrDis = 1.0f / ((i + 1) + 1.0f / config.max_corresp_size_init);
          //float newMaxCorrDis = align.getMaxCorrespondenceDistance () - (config.total_iterations - i) * 2 * config.max_corresp_size_init / (config.total_iterations * (config.total_iterations - 1));
          double newMaxCorrDis = mUtils::drop_acc_even(align.getMaxCorrespondenceDistance (), config.max_corresp_size_init, i, config.total_iterations);
          
          if(newMaxCorrDis >= 0)
          {
            align.setMaxCorrespondenceDistance (newMaxCorrDis);
            PCL_INFO ("New corres. distance: %f\n", newMaxCorrDis);
          }
        }
          
        prev = align.getLastIncrementalTransformation ();

        // visualize current state
        showCloudsRight(scene, object);
      }
      else
      {
        pcl::console::print_error ("Alignment failed!\n");
      }
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