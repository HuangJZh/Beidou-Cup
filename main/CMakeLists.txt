cmake_minimum_required(VERSION 3.5)
project(faim)

# Make sure that custom modules like FindXXXX are found
# Prefer to use system installation of gflags/glog
#set(GFLAGS_PREFER_EXPORTED_GFLAGS_CMAKE_CONFIGURATION TRUE)
#set(GLOG_PREFER_EXPORTED_GLOG_CMAKE_CONFIGURATION TRUE)
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake_modules)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -g -march=native")

message(STATUS "==============================================================")
message(STATUS "====================  Dependencies ===========================")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

#find_package(TBB REQUIRED)
#include_directories(${TBB_INCLUDE_DIR})
#message(STATUS "TBB_INTERACTIVE_VERSION: ${TBB_INTERACTIVE_VERSION}")
#if (TBB_INTERACTIVE_VERSION LESS 11004)
#	# enable global_control header for earlier TBB versions (ubuntu 16.04, 18.04)
#	add_definitions(-DTBB_PREVIEW_GLOBAL_CONTROL)
#endif()

#find_package(Gflags REQUIRED)
#find_package(Glog 0.3.5 REQUIRED)
#find_package(Eigen REQUIRED)
set(ADDITIONAL_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR}/unsupported)
find_package(GTSAM REQUIRED)



add_library(${PROJECT_NAME} SHARED
main/init.cpp
main/shutdown.cpp
objects/threaded_object.cpp
objects/fifo_imu.cpp
objects/fifo_cam.cpp
objects/fifo_fea.cpp
objects/source_imu.cpp
objects/source_cam.cpp
objects/acquisition.cpp
objects/sv_select.cpp
objects/telemetry.cpp
objects/pvt.cpp
objects/correlator.cpp
accessories/misc.cpp
accessories/PinholeCamera.cpp
includes/threaded_object.h
includes/fifo_imu.h
includes/fifo_cam.h
includes/fifo_fea.h
includes/source_imu.h
includes/source_cam.h
includes/sdr_structs.h
includes/protos.h
includes/includes.h
includes/globals.h
includes/config.h 
includes/signaldef.h
includes/acquisition.h
includes/sv_select.h
includes/telemetry.h
includes/pvt.h
includes/correlator.h 
#accessories/assert.h
accessories/calibration.h 
accessories/cameras.h
accessories/PinholeCamera.h
accessories/accumulator.h 
accessories/preintegration.h 
accessories/eigen_utils.h
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN_LIBS}
#${TBB_LIBRARIES}
#${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
#${PROJECT_SOURCE_DIR}/thirdparty/Sophus/lib/libSophus.so
/usr/lib/x86_64-linux-gnu/libtbb.so
gtsam
gtsam_unstable
fmt
)

target_include_directories(${PROJECT_NAME}
	PUBLIC
	${PROJECT_SOURCE_DIR}
	"/usr/include/tbb"
	"${PROJECT_SOURCE_DIR}/includes"
	${EIGEN_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
	"${PROJECT_SOURCE_DIR}/thirdparty/Sophus/include"
	"${PROJECT_SOURCE_DIR}/accessories"
	"${PROJECT_SOURCE_DIR}/thirdparty"
)
# Build examples

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/main)

add_executable(gps-sdr main/main.cpp)
target_link_libraries(gps-sdr PUBLIC faim pthread)



############################### gps-gse ##########################################


############################### gps-gse ##########################################

aux_source_directory(gse/src SRC_LIST)

find_package(wxWidgets COMPONENTS core base REQUIRED)
if (NOT wxWidgets_FOUND)
	message(FATAL_ERROR "wxWidgets not found.")
endif()
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} `wx-config --libs`")

include(${wxWidgets_USE_FILE})
include_directories(
	${EIGEN_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	includes
	gse
	gse/include
	"${PROJECT_SOURCE_DIR}/thirdparty/Sophus/include"
	"${PROJECT_SOURCE_DIR}/accessories"
	"${PROJECT_SOURCE_DIR}/thirdparty"
	)

add_executable(gps-gse ${SRC_LIST} gse/gui_classes.cpp)

target_link_libraries(gps-gse ${wxWidgets_LIBRARIES} ${OpenCV_LIBS})
