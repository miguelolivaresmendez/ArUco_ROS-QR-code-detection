cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

set(ARUCO_SOURCE_FILES
	src/aruco/arucofidmarkers.cpp
	src/aruco/board.cpp
	src/aruco/boarddetector.cpp
	src/aruco/cameraparameters.cpp
	src/aruco/cvdrawingutils.cpp
	src/aruco/marker.cpp
	src/aruco/markerdetector.cpp
	src/aruco/aruco_gl.cpp
)

set(ARUCO_HEADER_FILES
	src/aruco/aruco.h
	src/aruco/arucofidmarkers.h
	src/aruco/board.h
	src/aruco/boarddetector.h
	src/aruco/cameraparameters.h
	src/aruco/cvdrawingutils.h
	src/aruco/marker.h
	src/aruco/markerdetector.h
	src/aruco/exports.h
	src/aruco/aruco_gl.h
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
#link_directories(${PROJECT_SORCE_DIR}/lib/)
#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(aruco src/main.cpp ${ARUCO_SOURCE_FILES} ${OPENGL_LIBS})
rosbuild_add_executable(aruco_listener src/subscriber_aruco.cpp ${ARUCO_SOURCE_FILES} ${OPENGL_LIBS})
target_link_libraries(aruco glut GLU) #add an external library
target_link_libraries(aruco_listener glut GLU) #add an external library
