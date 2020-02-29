#include <pcReg/point_cloud_register.h>

namespace pcReg
{
    PointCloudRegister::PointCloudRegister()
    {

    }

    PointCloudRegister::~PointCloudRegister()
    {

    }

    const char* ToString(Algorithm v)
    {
      switch (v)
      {
          case icp:   return "icp";
          case prerejective:   return "prerejective";
          case fpcs: return "fpcs";
          default:      return "[Unknown Algorithm_type]";
      }
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Display source and target on the first viewport of the visualizer
     *
     */
    void PointCloudRegister::showCloudsLeft(const PointCloudT::Ptr cloud_target, const PointCloudT::Ptr cloud_source)
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
    void PointCloudRegister::showCloudsRight(const PointCloudNT::Ptr cloud_target, const PointCloudNT::Ptr cloud_source)
    {
      p->removePointCloud ("target");
      p->removePointCloud ("source");

      PointCloudColorHandlerGenericField<PointNT> tgt_color_handler (cloud_target, "curvature");
      if (!tgt_color_handler.isCapable ())
          PCL_WARN ("Cannot create curvature color handler!");

      PointCloudColorHandlerGenericField<PointNT> src_color_handler (cloud_source, "curvature");
      if (!src_color_handler.isCapable ())
          PCL_WARN ("Cannot create curvature color handler!");


      p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
      p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);

      p->spinOnce();
    }

    // ===============================================================================================
    

    void PointCloudRegister::RunRegister(int argc, char **argv, Algorithm alg)
    {
        // Load data
        std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
        loadPCD(argc, argv, data);

        // Check user input
        if (data.empty ())
        {
            PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
            PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
            return;
        }
        PCL_INFO ("Loaded %d datasets.", (int)data.size ());

        // Create a PCLVisualizer object
        p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
        p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
        p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

        PointCloudT::Ptr result (new PointCloudT), source, target;
        Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;

        for (std::size_t i = 1; i < data.size (); ++i)
        {
            source = data[i-1].cloud;
            target = data[i].cloud;

            // Add visualization data
            showCloudsLeft(source, target);
            PointCloudT::Ptr temp (new PointCloudT);
            PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
            
            switch (alg)
            {
                case Algorithm::icp:
                    pairAlign_icp(source, target, temp, pairTransform, true);
                    break;
                case Algorithm::prerejective:
                    pairAlign_prerej(source, target, temp, pairTransform, true);
                    break;
                
                default:
                    break;
            }

            //transform current pair into the global transform
            pcl::transformPointCloud (*temp, *result, GlobalTransform);

            //update the global transform
            GlobalTransform *= pairTransform;

            //save aligned pair, transformed into the first cloud's frame
            std::stringstream ss;
            ss << i << ".pcd";
            pcl::io::savePCDFile (config.data_path + "/merged_" + ss.str(), *result, true);

        }
    }
}
