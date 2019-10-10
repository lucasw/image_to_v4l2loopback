cmake_minimum_required(VERSION 2.8.3)
project(image_to_v4l2loopback)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	genmsg
	cv_bridge
	image_transport
	sensor_msgs
	)

find_package(PkgConfig REQUIRED COMPONENTS system)
find_package(OpenCV)

##############
## Coverage ##
##############

set(COVERAGE "OFF" CACHE STRING "Enable coverage generation.")

message(STATUS "Using COVERAGE: ${COVERAGE}")
if("${COVERAGE}" STREQUAL "ON")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
endif()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_to_v4l2loopback
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
	${OpenCV_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	)

add_executable(stream
  src/image_converter.cpp
  src/stream.cpp
  src/video_device.cpp
)

target_link_libraries(stream
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

set_property(TARGET stream APPEND_STRING PROPERTY COMPILE_FLAGS " -std=c++11")

#############
## Install ##
#############
install(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

##########
## Test ##
##########

if(CATKIN_ENABLE_TESTING)
  catkin_download_test_data(
    ${PROJECT_NAME}_sim-cam_2015-02-19-11-42-28.bag
    https://s3-us-west-1.amazonaws.com/ai-mf-data/sim-cam_2015-02-19-11-42-28.bag
    DESTINATION /tmp/${PROJECT_NAME}
    FILENAME sim-cam_2015-02-19-11-42-28.bag
    MD5 6927504331cd17d2ff8e9c498d0bf0ff
  )

  find_package(rostest REQUIRED)
  foreach(T
    tests/stream-test.launch
    )
    add_rostest(${T})
  endforeach()

  include_directories(src)
  catkin_add_gtest(unit_test
      tests/unit_test.cpp
      src/image_converter.cpp
      src/video_device.cpp
  )
  target_link_libraries(unit_test ${catkin_LIBRARIES})
  set_property(TARGET unit_test APPEND_STRING PROPERTY COMPILE_FLAGS " -std=c++11")
  target_link_libraries(stream
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES}
  )
endif()
