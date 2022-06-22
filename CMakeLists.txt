# MIT License

# Copyright (c) [year] [fullname]

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

#---------------------------------------------------------------------------------------------------------------------
cmake_minimum_required (VERSION "3.12" FATAL_ERROR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

project(dora_project VERSION 1.0.0 LANGUAGES CXX)
include_directories(include)
#---------------------------------------------------------------------------------------------------------------------
# General project configuration
file(GLOB_RECURSE DORA_HDR "include/dora/*.h" "include/dora/*.inl" "include/visualization_3d/*.h" "include/visualization_3d/*.inl")
file(GLOB_RECURSE DORA_SRC "src/dora/*.cpp" "src/visualization_3d/*.cpp")

set(APP_NAME "dora_exe")
add_executable(${APP_NAME} ${DORA_SRC} ${DORA_HDR} )

# Add boost
find_package(Boost 1.66 REQUIRED system exception iostreams)
target_include_directories(${APP_NAME} PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(${APP_NAME} PRIVATE ${Boost_LIBRARIES})

# Cartoghrapher
find_package(cartographer REQUIRED)
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
google_initialize_cartographer_project()
target_link_libraries(${APP_NAME} LINK_PUBLIC cartographer)

# Eigen3
find_package(Eigen3 REQUIRED)
target_link_libraries(${APP_NAME} LINK_PUBLIC Eigen3::Eigen)

# GLUT
find_package(GLUT REQUIRED)
target_link_libraries(${APP_NAME} LINK_PUBLIC ${GLUT_LIBRARIES} glut GLU GLEW pthread GL)

# Ouster
target_link_libraries(${APP_NAME} LINK_PUBLIC ouster_client)
target_include_directories(${APP_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/ouster)

# Yaml cpp
find_package(yaml-cpp REQUIRED)
target_include_directories(${APP_NAME} PUBLIC ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(${APP_NAME} LINK_PUBLIC ${YAML_CPP_LIBRARIES} yaml-cpp)

add_subdirectory(ouster)
