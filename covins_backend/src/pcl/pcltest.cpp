#include "pcl/pcltest.hpp"

#include <iostream>
#include <fstream>
#include <string>

#include "pcl/pclutil.hpp"

namespace pcl {

PCLTest::PCLTest(std::string data_path) : data_path(data_path) {
    std::cout << "PCLTest constructor" << std::endl;
}

PCLTest::~PCLTest() {
    std::cout << "PCLTest destructor" << std::endl;
}

void PCLTest::Run() {
    std::cout << "PCLTest run" << std::endl;

    std::fstream data_file;
    data_file.open(this->data_path, std::ios::in);

    std::string point_string;

    int num_points = 10;

    if (data_file.is_open()) {
        std::getline(data_file, point_string);
        CreateMap("test_map");
        while(getline(data_file, point_string, '\n') && num_points > 0) {
            // std::cout << point_string << std::endl;

            point_string.erase(0, point_string.find(',') + 1);
            std::string px = point_string.substr(0, point_string.find(','));
            point_string.erase(0, point_string.find(',') + 1);
            std::string py = point_string.substr(0, point_string.find(','));
            point_string.erase(0, point_string.find(',') + 1);
            std::string pz = point_string.substr(0, point_string.find(','));
            // point_string.erase(0, point_string.find(',') + 1);

            // double px = num_points;
            // double py = num_points;
            // double pz = num_points;

            // std::cout << std::to_string(px) + " " + std::to_string(py) + " " + std::to_string(pz) << std::endl;
            std::cout << px + " " + py + " " + pz << std::endl;

            InsertMapPoint(std::stod(px), std::stod(py), std::stod(pz), num_points, "test_map");
            // InsertMapPoint(px, py, pz, "test_map");

            num_points--;
        }
    }
    else {
        std::cout << "Unable to open file" << std::endl;
    }

    GetMapPoint("test_map");
    DeleteMapPoint(1, "test_map");
    GetMapPoint("test_map");
    // GetMapSchema("test_map");
}

}

