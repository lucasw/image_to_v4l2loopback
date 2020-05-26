cmake_minimum_required(VERSION 3.0)
project(image_to_v4l2loopback VERSION 0.1.0)

add_compile_options(-std=c++1z)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	genmsg
	image_transport
	roslint
	sensor_msgs
	std_msgs
  roscpp
	)

set(
  ROSLINT_CPP_OPTS
  "--extensions=cpp,h,hpp" "--filter=-whitespace/braces,-runtime/references"
)

roslint_cpp()

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
  include
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
  target_link_libraries(stream
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES}
  )
endif()
