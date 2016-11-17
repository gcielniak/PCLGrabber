cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(PCLGrabber)

set (EXTRA_INCLUDE_DIRS "")
set (EXTRA_LIBRARY_DIRS "")
set (EXTRA_LIBRARIES "")

if(WIN32)
ADD_DEFINITIONS(/bigobj)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /bigobj")
SET(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} /bigobj")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /bigobj")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /bigobj")

  if(EXISTS "$ENV{KINECTSDK20_DIR}")
    set(EXTRA_INCLUDE_DIRS "$ENV{KINECTSDK20_DIR}/inc")
    set(EXTRA_LIBRARY_DIRS "$ENV{KINECTSDK20_DIR}/lib/x64/")
    set(EXTRA_LIBRARIES "$ENV{KINECTSDK20_DIR}/lib/x64/kinect20.lib")
    add_definitions(-DHAVE_KINECT2_NATIVE)
  endif()
endif()

find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization)

if (false)
set(OpenCV_STATIC OFF)
find_package( OpenCV QUIET )

if (OPENCV_FOUND)
add_definitions(-DHAVE_OPENCV)
set (EXTRA_INCLUDE_DIRS ${EXTRA_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
set (EXTRA_LIBRARY_DIRS ${EXTRA_LIBRARY_DIRS} ${OpenCV_LIBS})
set (EXTRA_LIBRARIES ${EXTRA_LIBRARIES} ${OpenCV_LIBS})
endif()
endif()

# RealSenseGrabber library
find_path(RSGLIB_INCLUDE_DIR 
	NAMES real_sense_grabber.h
	PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../rs/include")

if(RSGLIB_INCLUDE_DIR)
	add_definitions(-DHAVE_REAL_SENSE)
	set (EXTRA_INCLUDE_DIRS ${EXTRA_INCLUDE_DIRS} ${RSGLIB_INCLUDE_DIR})
	set (EXTRA_LIBRARY_DIRS ${EXTRA_LIBRARY_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/../rs/build")
	set (EXTRA_LIBRARIES ${EXTRA_LIBRARIES} "real_sense")
endif()

include_directories(${PCL_INCLUDE_DIRS} ${EXTRA_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${EXTRA_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set (SOURCES
	src/BasicViewer.h
	src/DeviceInput.h
	src/EnsensoGrabberExt.h
	src/RealSenseGrabberExt.h
	src/FileGrabberExt.h
	src/FileOutput.h
	src/ImageReaderExt.h
	src/ImageUtils.h
	src/Kinect2NativeGrabber.h
	src/PCLGrabber.cpp)
	
add_executable(${CMAKE_PROJECT_NAME} ${SOURCES})
target_link_libraries(${CMAKE_PROJECT_NAME} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${EXTRA_LIBRARIES})
