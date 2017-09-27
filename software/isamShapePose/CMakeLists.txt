cmake_minimum_required(VERSION 3.6)
project(isamShapePose)

# cortire to speed up compilation
#set (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake")
#include(cotire)

set(CMAKE_BUILD_TYPE Debug)

add_definitions(
        -Wfatal-errors -g
)

if(NOT DEFINED ENV{CODE_BASE})
    set(ENV{CODE_BASE} $ENV{HOME}/push/)
endif()

if(NOT DEFINED ENV{SOFTWARE_BASE})
    set(ENV{SOFTWARE_BASE} $ENV{CODE_BASE}/software/)
endif()

set(INCLUDE_DIRS $ENV{SOFTWARE_BASE}/isamShapePose/include
                 $ENV{SOFTWARE_BASE}/build/include
                 $ENV{SOFTWARE_BASE}/externals/isam_v1_7/include
                 /usr/include/eigen3)
include_directories(${INCLUDE_DIRS})

set(LIBRARIES $ENV{SOFTWARE_BASE}/externals/isam_v1_7/lib/libisam.a
              /usr/local/lib/libbot2-lcmgl-client.so
              /usr/local/lib/liblcm.so
              /usr/lib/x86_64-linux-gnu/libcholmod.so
              $ENV{SOFTWARE_BASE}/build/lib/libjson_linux-gcc-5.4.0_libmt.so
              )

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES src/isamSP.cpp src/viz.cpp src/sp_util.cpp src/p_poly_dist.cpp src/EllipsoidApproxLimitSurface.cpp include/GPMotion.h src/GPMotion.cpp)

add_executable(isamShapePose ${SOURCE_FILES})
target_link_libraries(isamShapePose ${LIBRARIES})

set(SOURCE_FILES src/test.cpp)

add_executable(tester ${SOURCE_FILES})
target_link_libraries(tester ${LIBRARIES})


add_executable(test_push_measurement src/test_push_measurement.cpp)
target_link_libraries(test_push_measurement ${LIBRARIES})


add_executable(test_contact_measurement src/test_contact_measurement.cpp)
target_link_libraries(test_contact_measurement ${LIBRARIES})


set(SOURCE_FILES src/cppad_example/eigen_array.cpp src/viz.cpp src/sp_util.cpp src/p_poly_dist.cpp src/EllipsoidApproxLimitSurface.cpp)
add_executable(eigen_array ${SOURCE_FILES})
target_link_libraries(eigen_array ${LIBRARIES})


set(SOURCE_FILES src/opt_c_from_data.cpp src/viz.cpp src/sp_util.cpp src/p_poly_dist.cpp src/EllipsoidApproxLimitSurface.cpp)
add_executable(opt_c_from_data ${SOURCE_FILES})
target_link_libraries(opt_c_from_data ${LIBRARIES})
