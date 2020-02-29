#include <pcReg/point_cloud_register.h>
#include <mUtils/noise_reduce.h>

namespace pcReg
{
    // load stl and convert to PCD
    void PointCloudRegister::loadStl (int argc, char** argv, 
    std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
    {
      // TO DO
    }

    // load pcd directly
    void PointCloudRegister::loadPCD (int argc, char** argv, 
    std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
    {
      std::string extension (".pcd");
      // Suppose the first argument is the actual test model
      for (int i = 1; i < argc; i++)
      {

        std::string fname = std::string (config.data_path + "/" + argv[i]);

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

          //m.cloud = mUtils::NoiseReduce(m.cloud);
          
          if(config.is_reduce_noise)
          {
            pcl::StatisticalOutlierRemoval<PointT> sor;
            sor.setMeanK (50);
            sor.setStddevMulThresh (1.0);
            sor.setInputCloud (m.cloud);
            sor.filter (*m.cloud);
          }

          if(config.is_duplicate_source && i == 2) // transform point cloud 1 and save to target
          {
            // transform the source cloud by a large amount
            Eigen::Vector3f initial_offset (1.f, 0.f, 0.f);
            float angle = static_cast<float> (M_PI) / 2.f;
            Eigen::Quaternionf initial_rotation (std::cos (angle / 2.f), 0, 0, sin (angle / 2.f));
            PointCloudT cloud_source_transformed;
            pcl::transformPointCloud (*models[0].cloud, cloud_source_transformed, initial_offset, initial_rotation);
            *m.cloud = cloud_source_transformed;
          }

          models.push_back (m);
        }
      }
    }
}