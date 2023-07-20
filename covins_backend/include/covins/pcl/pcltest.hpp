#pragma once

#include <string>
#include <fstream>

namespace pcl {

class PCLTest {
public:

    std::string data_path;

    PCLTest(std::string data_path);
    ~PCLTest();

    void Run();
};

}