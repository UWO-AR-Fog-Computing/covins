#pragma once

#include <string>

#include <covins/covins_base/typedefs_base.hpp>

namespace pcl {
    const std::string MAP_PATH = "/root/rosworkspace/data/pcl/";

    void CreateMap(std::string map_id);

    void InsertMapPoint(covins::TypeDefs::precision_t px, covins::TypeDefs::precision_t py, covins::TypeDefs::precision_t pz, size_t id, std::string map_id);

    #ifndef PCL_TEST
    void InsertLandmark(covins::TypeDefs::LandmarkPtr lmp, size_t id, std::string map_id);
    #endif

    void DeleteMapPoint(size_t id, std::string map_id);

    void GetMapPoint(std::string map_id);

    void GetMapSchema(std::string map_id);

    std::string GetMapName(std::string map_id);

    #ifndef PCL_TEST
    std::tuple<covins::TypeDefs::precision_t, covins::TypeDefs::precision_t, covins::TypeDefs::precision_t> LandmarkToPos(covins::TypeDefs::LandmarkPtr lmp);
    #endif
}