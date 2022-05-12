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
# General project configuration
file(GLOB_RECURSE DORA_HDR "include/dora/*.h" "include/dora/*.inl")
file(GLOB_RECURSE DORA_SRC "src/dora/*.cpp")

set(APP_NAME "dora")
add_executable(${APP_NAME} ${DORA_SRC} ${DORA_HDR})
set_target_properties(${APP_NAME} PROPERTIES FOLDER "apps")
                                
target_include_directories(${APP_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )
    GroupSources(include)
    GroupSources(src)

    set_target_properties(${APP_NAME} PROPERTIES CXX_STANDARD ${VES_CPP_STD})

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