#include <perception/point_cloud_register.h>

// ===============================================================================================

int main(int argc, char** argv)
{
    // ROS params
    ros::init(argc, argv, "register_cloud");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    perception::PointCloudRegister registerApp;

    //registerApp.LoadParam();

    //registerApp.RunRegister(argc, argv);

    return 0;
}
