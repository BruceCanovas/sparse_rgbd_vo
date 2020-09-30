cmake_minimum_required(VERSION 2.8.3)
project(sparse_rgbd_vo)

add_compile_options(--std=c++11)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    image_transport
    geometry_msgs
    message_filters
    tf
    nav_msgs
    visualization_msgs
    pcl_ros
    tf_conversions
)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(OpenCV 3 REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(G2O REQUIRED)
set(G2O_LIBRARIES ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_TYPES_SBA} ${G2O_TYPES_SLAM3D} ${G2O_SOLVER_PCG})

catkin_package(
    CATKIN_DEPENDS message_runtime roscpp cv_bridge image_transport geometry_msgs message_filters tf nav_msgs visualization_msgs pcl_ros tf_conversions
    )

include_directories(
    include
    ${PROJECT_SOURCE_DIR}/third_party/eigen3/
    ${PROJECT_SOURCE_DIR}/third_party/ORB/
    ${PROJECT_SOURCE_DIR}/third_party/GMS/
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_library(orb
    ${PROJECT_SOURCE_DIR}/third_party/ORB/ORBextractor.cpp
)
target_link_libraries(orb
    ${OpenCV_LIBS}
)

add_library(gms
    ${PROJECT_SOURCE_DIR}/third_party/GMS/gms_matcher.cpp
)
target_link_libraries(gms
    ${OpenCV_LIBS}
)

add_library(sparse_rgbd_vo
    src/sparse_rgbd_vo.cpp
    src/local_map.cpp
    src/pnp_solver.cpp
)
target_link_libraries(sparse_rgbd_vo
    orb
    gms
    ${OpenCV_LIBS}
    ${G2O_LIBRARIES}
)

add_executable(sparse_rgbd_vo_node
     src/main.cpp 
     src/sparse_rgbd_vo_node.cpp
     )
target_link_libraries(sparse_rgbd_vo_node
    sparse_rgbd_vo
    ${catkin_LIBRARIES}
    )