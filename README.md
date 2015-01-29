ros-virtual-camera
==================

ROS node for streaming an image topic to  a video capture device. Mostly
based on this:

* [virtual_camera](https://github.com/czw90130/virtual_camera)

dev
===

```bash
$ git clone git@github.com:ixirobot/ros-virtual-camera.git
$ mkdir ros-virtual-camera-build
$ cd ros-virtual-camera-build
$ cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../ros-virtual-camera/
```

video devices
=============

Stream sink needs to a video capture device. These will be different depending
on you system.

Linx
---

Uses [v4l2-loopback](https://github.com/umlaeute/v4l2loopback).

```bash
$ sudo apt-get install v4l2loopback-*
$ sudo modprobe v4l2loopback video_nr=1 card_label="Loopback video device 0"
```

Mac
---
sol

Windows
---
sol

usage
=====

Typically just:

```bash
$ rosrun virtual_camera stream  /dev/video1  -s 640x480 -f YV12 image:=/my_camera/image
...
```

where `image:=/downward_looking_camera/image_raw` is the image topic to stream
to `/dev/video1`.