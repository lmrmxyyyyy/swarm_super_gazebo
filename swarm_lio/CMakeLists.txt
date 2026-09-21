cmake_minimum_required(VERSION 2.8.3)
project(swarm_lio)

SET(CMAKE_BUILD_TYPE "Release")

ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" ) 

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver2
  mavros_msgs
  message_generation
  eigen_conversions
  swarm_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
set(SWARMLIO_GTSAM_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../.deps/gtsam-4.1.1")
find_package(GTSAM REQUIRED
  PATHS "${SWARMLIO_GTSAM_PREFIX}/lib/cmake/GTSAM")

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIRS}
  include)

add_message_files(
  FILES
  Pose6D.msg
  States.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs mavros_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

add_executable(swarm_lio
        src/laserMapping.cpp
        include/ikd-Tree/ikd_Tree.cpp
        src/preprocess.cpp
        src/MultiUAV.cpp
        )

add_dependencies(swarm_lio
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(swarm_lio
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${PYTHON_LIBRARIES}
        ${GTSAM_LIBRARIES}
        gtsam
        -ltbb
        )
set_target_properties(swarm_lio PROPERTIES
        LINK_FLAGS "-Wl,--disable-new-dtags"
        BUILD_RPATH "${SWARMLIO_GTSAM_PREFIX}/lib"
        INSTALL_RPATH "${SWARMLIO_GTSAM_PREFIX}/lib")
target_include_directories(swarm_lio PRIVATE ${PYTHON_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
        scripts/px4_vision_fusion.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
