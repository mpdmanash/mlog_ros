cmake_minimum_required(VERSION 2.8.3)
project(vlog)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  roscpp
  rospy
  std_msgs
  std_srvs
  visualization_msgs
  nav_msgs
  cv_bridge
  tf
  tf_conversions
  cmake_modules # For Eigen and declare before Eigen
)

find_package(Ceres REQUIRED)

INCLUDE_DIRECTORIES ( "/home/manash/Codes/downloaded/eigen-eigen-67e894c6cd8f" )
#find_package(Eigen REQUIRED COMPONENTS core geometry)

find_package(OpenCV REQUIRED core highgui imgproc)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mav_perception
  CATKIN_DEPENDS message_filters roscpp rospy std_msgs cv_bridge nav_msgs
  DEPENDS OpenCV OpenMP
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -msse3" )

set (MAIN_INCLUDE_DIR include)

include_directories(${MAIN_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${OpenMP_INCLUDE_DIRS}
                    ${CERES_INCLUDE_DIRS}
)

add_executable(localize src/localize.cpp 
                        src/vlog.cpp)
target_link_libraries(localize ${catkin_LIBRARIES}
                                        ${OpenCV_LIBRARIES}
                                        ${OpenMP_LIBRARIES}
                                        ${CERES_LIBRARIES}
)

add_executable(gt src/gt.cpp)
target_link_libraries(gt ${catkin_LIBRARIES}
)
