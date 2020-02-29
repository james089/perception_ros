#include <pcReg/config.h>

namespace pcReg
{
    void LoadParam(Config &config)
    {
        ros::NodeHandle ph("~");
        if(ph.getParam("data_path",config.data_path) &&
            ph.getParam("voxel_grid_leaf_size",config.voxel_grid_leaf_size) &&
            ph.getParam("norm_est_k_search_val",config.norm_est_k_search_val) &&
            ph.getParam("max_corresp_size_init",config.max_corresp_size_init) &&
            ph.getParam("max_corresp_size_drop_step",config.max_corresp_size_drop_step) &&
            ph.getParam("total_iterations",config.total_iterations) )
        {
          ROS_INFO_STREAM("Loaded application parameters");
        }
        else
        {
          ROS_ERROR_STREAM("Failed to load application parameters");
        }
    }

}
