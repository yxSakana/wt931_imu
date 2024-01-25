#include "wt931/ImuController.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "imu_node");
    ImuController handle = ImuController();
    handle.start();
    return 0;
}