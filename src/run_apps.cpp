#include <pcReg/point_cloud_register.h>

using namespace pcReg;
// ===============================================================================================

int main(int argc, char** argv)
{
    // ROS params
    ros::init(argc, argv, "register_cloud");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    PointCloudRegister registerApp;

    registerApp.LoadParam();

    Algorithm al = (Algorithm)registerApp.config.type_reg_algorithm;
    cout << "Using algorithm: " << ToString(al) << endl;
    registerApp.RunRegister(argc, argv, al);

    return 0;
}
