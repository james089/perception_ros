#include <mUtils/noise_reduce.h>

namespace mUtils
{
    /// this may not work...
    mCloudT::Ptr NoiseReduce(mCloudT::Ptr cloud_input)
    {
        mCloudT::Ptr ptr_cloud_prcd (new mCloudT());
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
        sor.setInputCloud (cloud_input);
        sor.filter (*ptr_cloud_prcd);

        return ptr_cloud_prcd;
    }
}

