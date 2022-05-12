cmake_minimum_required(VERSION 3.1.0)

# ==== Project Name ====
project(ouster_client VERSION 0.2.0)
set(ouster_client_VERSION_SUFFIX "")

# ==== Requirements ====
find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)

# ==== Libraries ====
add_library(ouster_client src/client.cpp src/types.cpp src/netcompat.cpp src/lidar_scan.cpp src/image_processing.cpp)
target_link_libraries(ouster_client PUBLIC jsoncpp_lib Eigen3::Eigen)
if(WIN32)
  target_link_libraries(ouster_client PUBLIC ws2_32)
endif()
target_include_directories(ouster_client PUBLIC include)
target_include_directories(ouster_client SYSTEM PUBLIC include/optional-lite)