#---------------------------------------------------------------------------------------------------------------------
#  Vertical Engineering Solutions
#---------------------------------------------------------------------------------------------------------------------
# 
#  Copyright 2020 Vertical Engineering Solutions  - All Rights Reserved
# 
#  Unauthorized copying of this file, via any medium is strictly prohibited Proprietary and confidential.
# 
#  All information contained herein is, and remains the property of Vertical Engineering Solutions.  The 
#  intellectual and technical concepts contained herein are proprietary to Vertical Engineering Solutions 
#  and its suppliers and may be covered by UE and Foreign Patents, patents in process, and are protected 
#  by trade secret or copyright law. Dissemination of this information or reproduction of this material is 
#  strictly forbidden unless prior written permission is obtained from Vertical Engineering Solutions.
#
#---------------------------------------------------------------------------------------------------------------------
#
#  Maintainer: acasado@vengineerings.com
#
#---------------------------------------------------------------------------------------------------------------------


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