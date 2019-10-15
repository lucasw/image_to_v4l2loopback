# image_to_v4l2loopback [![Build Status](https://travis-ci.org/lucasw/image_to_v4l2loopback.svg?branch=master)](https://travis-ci.org/lucasw/image_to_v4l2loopback) [![Coverage Status](https://coveralls.io/repos/lucasw/image_to_v4l2loopback/badge.svg?branch=master)](https://coveralls.io/r/lucasw/image_to_v4l2loopback?branch=master)

ROS node for streaming an image topic to  a video capture device. Mostly
based on this:

* [virtual_camera](https://github.com/czw90130/virtual_camera)

## dev

Setup a workspace:

```bash
cd ~/catkin_ws/src  # or wherever catkin_ws is
git clone https://github.com/lucasw/image_to_v4l2loopback
cd ..
catkin build image_to_v4l2loopback
```

## video devices

Stream node needs to write to a video capture device, which varies depending on
your system.

### linux

Uses [v4l2-loopback](https://github.com/umlaeute/v4l2loopback):

Secure boot uefi will prevent the modprobe from working, try https://askubuntu.com/questions/762254/why-do-i-get-required-key-not-available-when-install-3rd-party-kernel-modules

```bash
sudo apt install mokutil
sudo mokutil --disable-validation
sudo reboot
```

```bash
sudo apt-get install v4l2loopback-*
sudo modprobe v4l2loopback video_nr=1
v4l2-ctl -D -d /dev/video1
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

## usage

### stream

Typically just:

```bash
rosrun image_to_v4l2loopback stream _device:=/dev/video1 _width:=640 _height:=480 _fourcc:=YV12 image:=/my_camera/image
```

where:

* `/dev/video1` target device
* `640x480` target size
* `YV12` target [pixel format](http://en.wikipedia.org/wiki/FourCC)
* `/my_camera/image` the source `sensor_msgs/Image` topic re-mapped to `image`

For more:

```bash
rosrun image_to_v4l2loopback stream --help
```

## tests

```bash
v4l2-ctl -D -d /dev/video1
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
ROS_VIRTUAL_CAM_STREAM_TEST_DEVICE=/dev/video1 catkin_make run_tests
```

If you want coverage:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Debug -DCOVERAGE=ON
ROS_VIRTUAL_CAM_STREAM_TEST_DEVICE=/dev/video1 catkin_make run_tests
lcov --path . --directory . --capture --output-file coverage.info
lcov --remove coverage.info 'tests/*' '/usr/*' '/opt/*' --output-file coverage.info
lcov --list coverage.info
```
