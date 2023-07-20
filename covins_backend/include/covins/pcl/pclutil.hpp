#pragma once

#include <string>

namespace pcl {
    const std::string MAP_PATH = "/root/rosworkspace/data/pcl/";

    void CreateMap(std::string map_id);

    void InsertMapPoint(double px, double py, double pz, size_t id, std::string map_id);

    void DeleteMapPoint(size_t id, std::string map_id);

    void GetMapPoint(std::string map_id);

    void GetMapSchema(std::string map_id);

    std::string GetMapName(std::string map_id);
}