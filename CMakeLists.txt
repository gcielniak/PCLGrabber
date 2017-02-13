cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(PCLGrabber)

set (EXTRA_INCLUDE_DIRS include)
set (EXTRA_LIBRARY_DIRS "")
set (EXTRA_LIBRARIES "")

# RealSenseGrabber library
# the built in version in PCL seems to be buggy for some reason
# try to find stand alone library first
# has to be called before PCL
find_path(RSGLIB_INCLUDE_DIR 
	NAMES real_sense_grabber.h
	PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../rs/include")

if(RSGLIB_INCLUDE_DIR)
	add_definitions(-DHAVE_REAL_SENSE)
	set (EXTRA_INCLUDE_DIRS ${EXTRA_INCLUDE_DIRS} ${RSGLIB_INCLUDE_DIR})
	set (EXTRA_LIBRARY_DIRS ${EXTRA_LIBRARY_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/../rs/build")
	set (EXTRA_LIBRARIES ${EXTRA_LIBRARIES} real_sense)
endif()

#PCL
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
find_package(OpenCV REQUIRED)

if (OPENCV_FOUND)
add_definitions(-DHAVE_OPENCV)
set (EXTRA_INCLUDE_DIRS ${EXTRA_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
set (EXTRA_LIBRARY_DIRS ${EXTRA_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})
set (EXTRA_LIBRARIES ${EXTRA_LIBRARIES} ${OpenCV_LIBS})
endif()

# RealSense SDK PCL support
if(!RSGLIB_INCLUDE_DIR)
if(EXISTS "${PCL_ROOT}/include/pcl-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}/pcl/io/real_sense_grabber.h")
	add_definitions(-DHAVE_REAL_SENSE)
endif()
endif()

# ZED camera support

if (DEFINED ENV{ZED_INCLUDE_DIRS})
    add_definitions(-DHAVE_ZED)

IF(WIN32) # Windows
	set (EXTRA_INCLUDE_DIRS ${EXTRA_INCLUDE_DIRS} $ENV{ZED_INCLUDE_DIRS})
	set (EXTRA_LIBRARY_DIRS ${EXTRA_LIBRARY_DIRS} $ENV{ZED_LIBRARY_DIR})
	set (EXTRA_LIBRARIES ${EXTRA_LIBRARIES} $ENV{ZED_LIBRARIES_64})
ELSE() # Linux
    find_package(ZED 1.0 REQUIRED)
    find_package(Eigen3 REQUIRED)
    
    ##For Jetson, OpenCV4Tegra is based on OpenCV2.4
    exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
    if ( CMAKE_SYSTEM_NAME2 MATCHES "aarch64" OR CMAKE_SYSTEM_NAME2 MATCHES "armv7l" )
        SET(OCV_VERSION "2.4")
        SET(CUDA_VERSION "")
    endif()
ENDIF(WIN32)

find_package(CUDA ${CUDA_VERSION} REQUIRED)

set (EXTRA_INCLUDE_DIRS ${EXTRA_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
set (EXTRA_LIBRARY_DIRS ${EXTRA_LIBRARY_DIRS} ${CUDA_LIBRARY_DIRS})

endif ()

# Stemmer Library: GenICam driver
if(EXISTS "$ENV{CVB}/Lib/C")
    add_definitions(-DHAVE_GENICAM)
    set(EXTRA_INCLUDE_DIRS ${EXTRA_INCLUDE_DIRS} "$ENV{CVB}/Lib/C")
    set(EXTRA_LIBRARY_DIRS ${EXTRA_LIBRARY_DIRS} "$ENV{CVB}/Lib/C")
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
	include/ZEDGrabber.h
	include/GenICamGrabber.h
	src/PCLGrabber.cpp)

add_executable(${CMAKE_PROJECT_NAME} ${SOURCES})
target_link_libraries(${CMAKE_PROJECT_NAME} ${EXTRA_LIBRARIES})
