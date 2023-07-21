cmake_minimum_required(VERSION 2.8.3)
project(covins_backend)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen3 3.3.4 EXACT REQUIRED)

# /root/rosworkspace/devel:/opt/ros/noetic

list(APPEND CMAKE_PREFIX_PATH "/tmp/TileDB/build/dist")
# find_package(TileDB REQUIRED)
# find_path(TILEDB_HEADER tiledb)


# find_path(HIREDIS_HEADER hiredis)
# find_library(HIREDIS_LIB hiredis)
# find_path(REDIS_PLUS_PLUS_HEADER sw)
# find_library(REDIS_PLUS_PLUS_LIB redis++)

add_compile_options(-std=c++17 -pthread -pipe -Wall -Wextra -pedantic -Wno-unused-parameter -Wno-unused-function -Wno-vla)
add_compile_options(-w)

#ROS_BUILD_TYPE: Set the build type. Options are (default: RelWithDebInfo):
#Debug : w/ debug symbols, w/o optimization
#Release : w/o debug symbols, w/ optimization
#RelWithDebInfo : w/ debug symbols, w/ optimization
#RelWithAsserts : w/o debug symbols, w/ optimization, w/ assertions (i.e., w/o -DNDEBUG). New in ros 1.1.
#MinSizeRel : w/o debug symbols, w/ optimization, stripped binaries

IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF()

include_directories(
    "include/covins"
    ${PROJECT_SOURCE_DIR}/thirdparty
)

set(EXTERNAL_LIBRARIES
  pthread
)

set(BASE_SOURCE_FILES
    src/covins_base/keyframe_base.cpp
    src/covins_base/landmark_base.cpp
    src/covins_base/map_base.cpp
    src/covins_base/visualization_base.cpp

    # config
    src/covins_base/config_backend.cpp
)

set(BASE_HEADER_FILES
    include/covins/covins_base/keyframe_base.hpp
    include/covins/covins_base/landmark_base.hpp
    include/covins/covins_base/map_base.hpp
    include/covins/covins_base/optimization_base.hpp
    include/covins/covins_base/visualization_base.hpp

    # config
    include/covins/covins_base/config_backend.hpp
)

set(BACKEND_SOURCE_FILES
    src/covins_backend/backend.cpp
    src/covins_backend/communicator_be.cpp
    src/covins_backend/feature_matcher_be.cpp
    src/covins_backend/handler_be.cpp
    src/covins_backend/keyframe_be.cpp
    src/covins_backend/landmark_be.cpp
    src/covins_backend/kf_database.cpp
    src/covins_backend/map_be.cpp
    src/covins_backend/optimization_be.cpp
    src/covins_backend/placerec_be.cpp
    src/covins_backend/placerec_gen_be.cpp
    src/covins_backend/RelNonCentralPosSolver.cpp
    src/covins_backend/Se3Solver.cpp
    src/covins_backend/visualization_be.cpp

    # Matcher
    src/matcher/MatchingAlgorithm.cpp
    src/matcher/ImageMatchingAlgorithm.cpp
    src/matcher/LandmarkMatchingAlgorithm.cpp
    src/matcher/opengv/frame-relative-adapter.cpp
    src/matcher/opengv/FrameNoncentralRelativeAdapter.cpp

    # Dense Matcher
    src/dense_matcher/DenseMatcher.cpp
    src/dense_matcher/ThreadPool.cpp

    # PCL
    # src/covins_backend/pclutil.cpp
    src/pcl/pclutil.cpp
)

set(BACKEND_HEADER_FILES
    include/covins/covins_backend/backend.hpp
    include/covins/covins_backend/communicator_be.hpp
    include/covins/covins_backend/feature_matcher_be.hpp
    include/covins/covins_backend/handler_be.hpp
    include/covins/covins_backend/keyframe_be.hpp
    include/covins/covins_backend/landmark_be.hpp
    include/covins/covins_backend/kf_database.hpp
    include/covins/covins_backend/map_be.hpp
    include/covins/covins_backend/optimization_be.hpp
    include/covins/covins_backend/placerec_be.hpp
    include/covins/covins_backend/placerec_gen_be.hpp
    include/covins/covins_backend/Se3Solver.h
    include/covins/covins_backend/RelNonCentralPosSolver.hpp
    include/covins/covins_backend/visualization_be.hpp

    # Base
    include/covins/covins_base/placerec_base.hpp
    include/covins/covins_base/vocabulary.h

    # Matcher
    include/covins/matcher/MatchingAlgorithm.h
    include/covins/matcher/ImageMatchingAlgorithm.h
    include/covins/matcher/LandmarkMatchingAlgorithm.h

    include/covins/matcher/opengv/rel_pose/frame-relative-adapter.hpp
    include/covins/matcher/opengv/rel_pose/FrameNoncentralRelativeAdapter.hpp
    include/covins/matcher/opengv/sac_problems/frame-relative-pose-sac-problem.hpp

    # Dense Matcher
    include/covins/dense_matcher/DenseMatcher.hpp
    include/covins/dense_matcher/ThreadPool.hpp

    # BRISK
    include/covins/brisk/macros.h
    include/covins/brisk/hamming.h
    include/covins/brisk/hamming-inl.h
    include/covins/brisk/neon-helpers.h

    # PCL
    # include/covins/covins_backend/pclutil.hpp
    include/covins/pcl/pclutil.hpp
)

set(TILEDB_HEADERS "/tmp/TileDB/build/dist/include")

set(PCL_TEST_HEADER_FILES
    include/covins/pcl/pcltest.hpp
    include/covins/pcl/pclutil.hpp
    # include/covins/covins_backend/landmark_be.hpp
)
set(PCL_TEST_SOURCE_FILES
    src/pcl/pcltest.cpp
    src/pcl/pclutil.cpp
    # src/covins_backend/landmark_be.cpp
)

if (NOT BUILD_LIBRARY)
    # cs_add_library(covins_base ${BASE_SOURCE_FILES} ${BASE_HEADER_FILES})
    # target_link_libraries(covins_base ${catkin_LIBRARIES} ${EXTERNAL_LIBRARIES})

    # cs_add_library(covins_backend ${BACKEND_SOURCE_FILES} ${BACKEND_HEADER_FILES})
    # target_include_directories(covins_backend PUBLIC ${TILEDB_HEADERS})
    # target_link_libraries(covins_backend covins_base ${PROJECT_SOURCE_DIR}/thirdparty/DBoW2/lib/libDBoW2.so /tmp/TileDB/build/dist/lib/libtiledb.so)
    # target_compile_definitions(covins_backend PRIVATE POINT_CLOUD_MAP=1)

    # cs_add_executable(covins_backend_node
    #     covins_sys/src/covins_backend_node.cpp
    # )
    # target_link_libraries(covins_backend_node covins_backend)


    cs_add_library(pcl_test ${PCL_TEST_SOURCE_FILES} ${PCL_TEST_HEADER_FILES})
    target_link_libraries(pcl_test ${catkin_LIBRARIES} ${EXTERNAL_LIBRARIES})

    target_include_directories(pcl_test PUBLIC ${TILEDB_HEADERS})
    target_link_libraries(pcl_test /tmp/TileDB/build/dist/lib/libtiledb.so)
    target_compile_definitions(pcl_test PRIVATE PCL_TEST=1)

    cs_add_executable(pcl_test_node
        covins_sys/src/pcl_node.cpp
    )
    target_link_libraries(pcl_test_node pcl_test)


    # target_link_libraries(covins_backend_node TileDB::tiledb_shared)
    # target_include_directories(covins_backend PUBLIC ${HIREDIS_HEADER})
    # target_link_libraries(covins_backend ${HIREDIS_LIB})
    # target_include_directories(covins_backend PUBLIC ${REDIS_PLUS_PLUS_HEADER})
    # target_link_libraries(covins_backend ${REDIS_PLUS_PLUS_LIB})

else()
    if (NOT USE_CATKIN)
        include_directories(${CMAKE_SOURCE_DIR}/thirdparty/cereal)
        # create library
        message("${PROJECT_SOURCE_DIR}/lib")
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

        add_library(${PROJECT_NAME} SHARED ${COMM_SOURCE_FILES} ${COMM_HEADER_FILES})
        option(BUILD_SHARED_LIBS "Build shared library" ON)

        target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
        target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})
        target_link_libraries(${PROJECT_NAME} ${GLOG_LIBRARIES})
    endif()

endif()

add_doxygen(NOT_AUTOMATIC)
