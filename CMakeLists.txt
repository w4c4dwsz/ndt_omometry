cmake_minimum_required(VERSION 3.0.0)
project(ndt_omometry VERSION 0.1.0)

SET(CMAKE_BUILD_TYPE "Debug")  # 定义编译类型
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb") # 定义Debug编译参数
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall") # 定义Release编译参数

find_package(PCL REQUIRED QUIET)
find_package (Eigen REQUIRED QUIET)
find_package(gflags REQUIRED)
find_package(OpenCV REQUIRED QUIET)
find_package(Ceres REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

add_executable(main main.cpp)
target_link_libraries(main ${PCL_LIBRARIES} ${gflags_LIBRARIES})


add_executable(ndt_test ndt_test.cpp)
target_link_libraries(ndt_test ${PCL_LIBRARIES} ${gflags_LIBRARIES} )


add_executable(ndt_resolution ndt_resolution.cpp)
target_link_libraries(ndt_resolution ${PCL_LIBRARIES} ${gflags_LIBRARIES})

add_executable(PCLvisual_test PCLvisual_test.cpp)
target_link_libraries(PCLvisual_test ${PCL_LIBRARIES} ${gflags_LIBRARIES})

add_executable(write_PCD write_PCD.cpp)
target_link_libraries(write_PCD ${PCL_LIBRARIES} ${gflags_LIBRARIES})

add_executable(planar_segmentation planar_segmentation.cpp)
target_link_libraries(planar_segmentation ${PCL_LIBRARIES} ${gflags_LIBRARIES})

add_executable(cluster_test cluster_test.cpp )
target_link_libraries(cluster_test ${PCL_LIBRARIES} ${gflags_LIBRARIES})

add_executable(GPF_test GPF_test.cpp )
target_link_libraries(GPF_test ${PCL_LIBRARIES} ${gflags_LIBRARIES})

add_executable(test test.cpp GPF.cpp GPF.h)
target_link_libraries(test ${PCL_LIBRARIES} ${gflags_LIBRARIES})

add_executable(RGF_test RGF_test.cpp RGF.cpp)
target_link_libraries(RGF_test ${PCL_LIBRARIES} ${gflags_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(point_line_icp_test point_line_icp_test.cpp)
target_link_libraries(point_line_icp_test ${PCL_LIBRARIES} ${gflags_LIBRARIES} ${OpenCV_LIBRARIES} Ceres::ceres)

add_executable(range_image range_image.cpp RGF.cpp)
target_link_libraries(range_image ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} Ceres::ceres ${gflags_LIBRARIES})

add_executable(icp_test icp_test.cpp RGF.cpp)
target_link_libraries(icp_test ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${gflags_LIBRARIES})