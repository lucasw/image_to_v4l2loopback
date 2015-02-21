# ros-virtual-camera [![Build Status](https://travis-ci.org/mayfieldrobotics/ros-virtual-camera.svg?branch=master)](https://travis-ci.org/mayfieldrobotics/ros-virtual-camera) [![Coverage Status](https://coveralls.io/repos/mayfieldrobotics/ros-virtual-camera/badge.svg?branch=master)](https://coveralls.io/r/mayfieldrobotics/ros-virtual-camera?branch=master)

ROS node for streaming an image topic to  a video capture device. Mostly
based on this:

* [virtual_camera](https://github.com/czw90130/virtual_camera)

## dev

Setup a workspace:

```bash
$ mkdir ~/ros/virtual-camera-ws/src -p
$ cd ~/ros/virtual-camera-ws/src
$ git clone git@github.com:ixirobot/ros-virtual-camera.git virtual_camera
$ catkin_init_workspace
$ cd ..
$ catkin_make
```

and a project if you want:

```bash
$ mkdir ~/ros/virtual-camera-prj -p
$ cd ~/ros/virtual-camera-prj
$ cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ~/ros/virtual-camera-ws/src/virtual_camera
```

## video devices

Stream node needs to write to a video capture device, which varies depending on
your system.

### linux

Uses [v4l2-loopback](https://github.com/umlaeute/v4l2loopback):

```bash
$ sudo apt-get install v4l2loopback-*
$ sudo modprobe v4l2loopback video_nr=1
$ v4l2-ctl -D -d /dev/video1
Driver Info (not using libv4l2):
    Driver name   : v4l2 loopback
    Card type     : Dummy video device (0x0000)
    Bus info      : v4l2loopback:0
    Driver version: 0.8.0
    Capabilities  : 0x05000003
        Video Capture
        Video Output
        Read/Write
        Streaming
```

### mac

**sol**

### windows

**sol**

## usage

### stream

Typically just:

```bash
$ rosrun virtual_camera stream /dev/video1 -s 640x480 -f YV12 image:=/my_camera/image
```

where:

* `/dev/video1` target device
* `640x480` target size
* `YV12` target [pixel format](http://en.wikipedia.org/wiki/FourCC)
* `/my_camera/image` the source `sensor_msgs/Image` topic re-mapped to `image`

For more:

```bash
$ rosrun virtual_camera stream --help
```

## tests

```bash
$ v4l2-ctl -D -d /dev/video1
Driver Info (not using libv4l2):
    Driver name   : v4l2 loopback
    Card type     : Dummy video device (0x0000)
    Bus info      : v4l2loopback:0
    Driver version: 0.8.0
    Capabilities  : 0x05000003
        Video Capture
        Video Output
        Read/Write
        Streaming
$ ROS_VIRTUAL_CAMERA_STREAM_TEST_DEVICE=/dev/video1 catkin_make run_tests
```
