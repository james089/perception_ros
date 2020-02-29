#include <pcReg/point_cloud_register.h>

namespace pcReg
{
    void PointCloudRegister::LoadParam()
    {
        ros::NodeHandle ph("~");
        if(ph.getParam("data_path",config.data_path) &&
            ph.getParam("type_reg_algorithm",config.type_reg_algorithm) &&
            ph.getParam("is_reduce_noise",config.is_reduce_noise) &&
            ph.getParam("is_duplicate_source",config.is_duplicate_source) &&
            ph.getParam("voxel_grid_leaf_size",config.voxel_grid_leaf_size) &&
            ph.getParam("norm_est_k_search_val",config.norm_est_k_search_val) &&
            ph.getParam("max_corresp_size_init",config.max_corresp_size_init) &&
            ph.getParam("total_iterations",config.total_iterations) &&
            ph.getParam("trans_epsilon",config.trans_epsilon) &&
            ph.getParam("max_sub_iteration",config.max_sub_iteration) &&
            ph.getParam("is_set_eucl_fit_epsl",config.is_set_eucl_fit_epsl) &&
            ph.getParam("euclidean_fitness_epsilon",config.euclidean_fitness_epsilon) &&
            ph.getParam("num_samples",config.num_samples) &&
            ph.getParam("corresp_randomness",config.corresp_randomness) &&
            ph.getParam("similarity_thresh",config.similarity_thresh))
        {
          ROS_INFO_STREAM("Loaded application parameters");
        }
        else
        {
          ROS_ERROR_STREAM("Failed to load application parameters");
        }
    }

}
