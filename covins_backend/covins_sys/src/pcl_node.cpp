
#include <iostream>
#include <csignal>
#include <cstdlib>
#include <thread>
#include <string>

#include <ros/ros.h>

#include "pcl/pcltest.hpp"


auto SignalHandler(int signum)->void {
   std::cout << "Interrupt signal (" << signum << ") received." << std::endl;
   exit(signum);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "PCL_Test_Node");
    signal(SIGINT, SignalHandler);

    std::string data_path = "/root/rosworkspace/data/datasets/EuRoC/MH01/mav0/leica0/data.csv";

    std::shared_ptr<pcl::PCLTest> pcl_test(new pcl::PCLTest(data_path));
    std::thread pcl_thread(&pcl::PCLTest::Run, pcl_test);

    ros::spin();

    return 0;
}