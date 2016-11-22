cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(PCLGrabber)

#PCL
set (EXTRA_INCLUDE_DIRS include)
set (EXTRA_LIBRARY_DIRS "")
set (EXTRA_LIBRARIES "")

find_package(PCL 1.3 REQUIRED)

set (EXTRA_INCLUDE_DIRS ${EXTRA_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
set (EXTRA_LIBRARY_DIRS ${EXTRA_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})
set (EXTRA_LIBRARIES ${EXTRA_LIBRARIES} ${PCL_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})

# Windows and Kinect 2 Native support
if(WIN32)
ADD_DEFINITIONS(/bigobj)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /bigobj")
SET(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} /bigobj")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /bigobj")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /bigobj")

  if(EXISTS "$ENV{KINECTSDK20_DIR}")
    set(EXTRA_INCLUDE_DIRS ${EXTRA_INCLUDE_DIRS} "$ENV{KINECTSDK20_DIR}/inc")
    set(EXTRA_LIBRARY_DIRS ${EXTRA_LIBRARY_DIRS} "$ENV{KINECTSDK20_DIR}/lib/x64/")
    set(EXTRA_LIBRARIES ${EXTRA_LIBRARIES} "$ENV{KINECTSDK20_DIR}/lib/x64/kinect20.lib")
    add_definitions(-DHAVE_KINECT2_NATIVE)
  endif()
endif()

#OpenCV support
if (false)
set(OpenCV_STATIC OFF)
find_package( OpenCV QUIET )

if (OPENCV_FOUND)
add_definitions(-DHAVE_OPENCV)
set (EXTRA_INCLUDE_DIRS ${EXTRA_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
set (EXTRA_LIBRARY_DIRS ${EXTRA_LIBRARY_DIRS} ${OpenCV_LIBS_DIR})
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
	set (EXTRA_LIBRARIES ${EXTRA_LIBRARIES} real_sense)
endif()

include_directories(${EXTRA_INCLUDE_DIRS})
link_directories(${EXTRA_LIBRARY_DIRS})

set (SOURCES
	include/BasicViewer.h
	include/DeviceInput.h
	include/EnsensoGrabberExt.h
	include/RealSenseGrabberExt.h
	include/FileGrabberExt.h
	include/FileOutput.h
	include/ImageReaderExt.h
	include/ImageUtils.h
	include/Kinect2NativeGrabber.h
	src/PCLGrabber.cpp)
	
add_executable(${CMAKE_PROJECT_NAME} ${SOURCES})
target_link_libraries(${CMAKE_PROJECT_NAME} ${EXTRA_LIBRARIES})
