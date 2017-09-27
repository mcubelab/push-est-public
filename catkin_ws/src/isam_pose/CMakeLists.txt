cmake_minimum_required(VERSION 2.8.3)
project(isam_pose)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW) # Required by CMake 2.7+
endif(COMMAND cmake_policy)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp  rospy  std_msgs  sensor_msgs  geometry_msgs  message_generation  apriltags  tf
)
find_package(
        Boost REQUIRED COMPONENTS program_options
)
function(JOIN VALUES GLUE OUTPUT)
    string (REGEX REPLACE "([^\\]|^);" "\\1${GLUE}" _TMP_STR "${VALUES}")
    string (REGEX REPLACE "[\\](.)" "\\1" _TMP_STR "${_TMP_STR}") #fixes escaping
    set (${OUTPUT} "${_TMP_STR}" PARENT_SCOPE)
endfunction()

catkin_package()

###########
## Build ##
###########

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_VERBOSE_MAKEFILE ON)

add_definitions(
        -Wfatal-errors
        -Wno-deprecated-declarations
)

if(NOT DEFINED ENV{CODE_BASE})
    set(ENV{CODE_BASE} $ENV{HOME}/push/)
endif()

if(NOT DEFINED ENV{SOFTWARE_BASE})
    set(ENV{SOFTWARE_BASE} $ENV{CODE_BASE}/software/)
endif()

set(INCLUDE_DIRS include
                 $ENV{SOFTWARE_BASE}/build/include
                 $ENV{SOFTWARE_BASE}/externals/isam_v1_7/include
                 /usr/include/eigen3)

set(catkin_INCLUDE_DIRS /home/mcube/push/catkin_ws/devel/include
        /home/mcube/push/catkin_ws/src/apriltags/include
        /opt/ros/kinetic/include
        /usr/include)  # hack remove opencv
include_directories(${INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

set(LIBRARIES $ENV{SOFTWARE_BASE}/externals/isam_v1_7/lib/libisam.a
              #/usr/local/lib/libbot2-lcmgl-client.so
              #/usr/local/lib/liblcm.so
              /usr/lib/x86_64-linux-gnu/libcholmod.so
              $ENV{SOFTWARE_BASE}/build/lib/libjson_linux-gcc-5.4.0_libmt.so
              )
JOIN("${catkin_INCLUDE_DIRS}" " " OUT)
Message("catkin_INCLUDE_DIRS " ${OUT})


set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES src/isam_pose.cpp  src/sp_util.cpp
        src/p_poly_dist.cpp
        src/RosMsgBuffer.cpp
        src/isam_pose_logger.cpp
        src/isam_pose_param.cpp
        src/Feeder.cpp
        #src/viz.cpp
        #src/GPMotion.cpp
        #src/EllipsoidApproxLimitSurface.cpp
        )

add_executable(isam_pose ${SOURCE_FILES})

add_dependencies(isam_pose ${catkin_EXPORTED_TARGETS})
find_package(catkin REQUIRED COMPONENTS
        roscpp  rospy  std_msgs  sensor_msgs  geometry_msgs  message_generation   tf
        )  # hack remove linking with opencv
target_link_libraries(isam_pose ${LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_library(sp_util src/sp_util.cpp)

