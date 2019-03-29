cmake_minimum_required(VERSION 3.13)
project(lidar_reconstruction_camera)

set(CMAKE_CXX_STANDARD 11)
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3 -fopenmp" )
set(CMAKE_BUILD_TYPE "Release")

# Eigen
include_directories( "/usr/include/eigen3" )
# OpenCV
find_package( OpenCV 3.4 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
# Sophus
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )
# pcl
find_package( PCL REQUIRED COMPONENT common io )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

set( THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(lidar_reconstruction_camera main.cpp find_feature_matches.cpp find_feature_matches.h "pose_estimation_2d2d .cpp" "pose_estimation_2d2d .h" triangulation.cpp triangulation.h pixel2cam.cpp pixel2cam.h pcl_cloud.cpp pcl_cloud.h epipolarSearch.cpp epipolarSearch.h NCC.cpp NCC.h)
target_link_libraries( lidar_reconstruction_camera ${THIRD_PARTY_LIBS} )