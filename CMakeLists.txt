cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(PCLGrabber)

set (EXTRA_INCLUDE_DIRS "")
set (EXTRA_LIBRARY_DIRS "")
set (EXTRA_LIBRARIES "")


if(WIN32)
  if(EXISTS "$ENV{KINECTSDK20_DIR}")
    set(EXTRA_INCLUDE_DIRS "$ENV{KINECTSDK20_DIR}/inc")
    set(EXTRA_LIBRARY_DIRS "$ENV{KINECTSDK20_DIR}/lib/x64/")
    set(EXTRA_LIBRARIES "$ENV{KINECTSDK20_DIR}/lib/x64/kinect20.lib")
  endif()
endif()

find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization)

include_directories(${PCL_INCLUDE_DIRS} ${EXTRA_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${EXTRA_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${CMAKE_PROJECT_NAME} src/pcl_grabber.cpp)
target_link_libraries(${CMAKE_PROJECT_NAME} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${EXTRA_LIBRARIES})