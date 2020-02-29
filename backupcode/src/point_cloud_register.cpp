#include <perception/point_cloud_register.h>

namespace perception
{
    PointCloudRegister::PointCloudRegister()
    {

    }

    PointCloudRegister::~PointCloudRegister()
    {

    }

    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Display source and target on the first viewport of the visualizer
     *
     */
    void PointCloudRegister::showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
    {
      p->removePointCloud ("vp1_target");
      p->removePointCloud ("vp1_source");

      PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
      PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
      p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
      p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

      PCL_INFO ("Press q to begin the registration.\n");
      p-> spin();
    }


    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Display source and target on the second viewport of the visualizer
     *
     */
    void PointCloudRegister::showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
    {
      p->removePointCloud ("source");
      p->removePointCloud ("target");

      PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature");
      if (!tgt_color_handler.isCapable ())
          PCL_WARN ("Cannot create curvature color handler!");

      PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature");
      if (!src_color_handler.isCapable ())
          PCL_WARN ("Cannot create curvature color handler!");


      p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
      p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);

      p->spinOnce();
    }

    // ===============================================================================================

    void PointCloudRegister::loadData (int argc, char** argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
    {
      std::string extension (".pcd");
      // Suppose the first argument is the actual test model
      for (int i = 1; i < argc; i++)
      {
        std::string fname = std::string (config.data_path + "/" + argv[i]);
        //cout << fname;
        // Needs to be at least 5: .plot
        if (fname.size () <= extension.size ())
          continue;

        std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

        //check that the argument is a pcd file
        if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
        {
          // Load the cloud and saves it into the global list of models
          PCD m;
          m.f_name = fname; //argv[i];
          pcl::io::loadPCDFile (fname, *m.cloud);//(argv[i], *m.cloud);
          //remove NAN points from the cloud
          std::vector<int> indices;
          pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

          models.push_back (m);
        }
      }
    }

    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Align a pair of PointCloud datasets and return the result
      * \param cloud_src the source PointCloud
      * \param cloud_tgt the target PointCloud
      * \param output the resultant aligned source PointCloud
      * \param final_transform the resultant transform between source and target
      */
    void PointCloudRegister::pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
    {
      //
      // Downsample for consistency and speed
      // \note enable this for large datasets
      PointCloud::Ptr src (new PointCloud);
      PointCloud::Ptr tgt (new PointCloud);
      pcl::VoxelGrid<PointT> grid;
      if (downsample)
      {
        grid.setLeafSize (0.05, 0.05, 0.05);
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
      PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
      PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

      pcl::NormalEstimation<PointT, PointNormalT> norm_est;
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
      norm_est.setSearchMethod (tree);
      norm_est.setKSearch (30);

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
      pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
      reg.setTransformationEpsilon (1e-6);
      // Set the maximum distance between two correspondences (src<->tgt) to 10cm
      // Note: adjust this based on the size of your datasets
      reg.setMaxCorrespondenceDistance (0.1);
      // Set the point representation
      reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

      reg.setInputSource (points_with_normals_src);
      reg.setInputTarget (points_with_normals_tgt);

      //
      // Run the same optimization in a loop and visualize the results
      Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
      PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
      reg.setMaximumIterations (2);
      for (int i = 0; i < 50; ++i)
      {
        PCL_INFO ("Iteration Nr. %d.\n", i);

        // save cloud for visualization purpose
        points_with_normals_src = reg_result;

        // Estimate
        reg.setInputSource (points_with_normals_src);
        reg.align (*reg_result);

            //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation () * Ti;

            //if the difference between this transformation and the previous one
            //is smaller than the threshold, refine the process by reducing
            //the maximal correspondence distance
        if (std::abs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
          reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

        prev = reg.getLastIncrementalTransformation ();

        // visualize current state
        showCloudsRight(points_with_normals_tgt, points_with_normals_src);
      }

        //
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