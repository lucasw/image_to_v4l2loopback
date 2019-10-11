/*
 * Copyright (c) 2013, Zhiwei Chu
 * Copyright (c) 2015, mayfieldrobotics.
 */

#include <fcntl.h>
#include <image_to_v4l2loopback/video_device.h>
#include <ros/ros.h>
#include <stdexcept>
#include <string.h>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

VideoDevice::VideoDevice(const std::string &path) : fd_(-1) {
  fd_ = open(path.c_str(), O_RDWR);
  if (fd_ == -1) {
    std::stringstream ss;
    ss << "open('" << path << "') failed - errno=" << errno << " ('"
       << strerror(errno) << "')";
    ROS_ERROR_STREAM(ss.str());
    throw std::runtime_error(ss.str());
  }
  ROS_INFO_STREAM("opened " << path << " with descriptor: " << fd_);
}

VideoDevice::VideoDevice(const VideoDevice &other) : fd_(-1) {
  fd_ = dup(other.fd_);
  if (fd_ == -1) {
    std::stringstream ss;
    ss << "dup('" << other.fd_ << "') failed - errno=" << errno << " ('"
       << strerror(errno) << "')";
    ROS_ERROR_STREAM(ss.str());
    throw std::runtime_error(ss.str());
  }
}

VideoDevice::~VideoDevice() {
  close(fd_);
  fd_ = -1;
}

int VideoDevice::stream_on() {
  int type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  int rc = ioctl(fd_, VIDIOC_STREAMON, &type);
  if (rc == -1) {
    ROS_ERROR("ioctl(%d, VIDIOC_STREAMON) failed - errno=%d, %s", fd_, errno,
              strerror(errno));
  }
  return rc;
}

int VideoDevice::stream_off() {
  int type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  int rc = ioctl(fd_, VIDIOC_STREAMOFF, &type);
  if (rc == -1) {
    ROS_ERROR("ioctl(%d, VIDIOC_STREAMOFF) failed - errno=%d, %s", fd_, errno,
              strerror(errno));
  }
  return rc;
}

int VideoDevice::capabilities(v4l2_capability &capability) {
  int rc = ioctl(fd_, VIDIOC_QUERYCAP, &capability);
  if (rc == -1) {
    ROS_ERROR("ioctl(%d, VIDIOC_QUERYCAP) failed - errno=%d, %s", fd_, errno,
              strerror(errno));
  }
  return rc;
}

int VideoDevice::get_format(v4l2_format &format) {
  int rc = ioctl(fd_, VIDIOC_G_FMT, &format);
  if (rc == -1) {
    ROS_ERROR("ioctl(%d, VIDIOC_G_FMT) failed - errno=%d, %s", fd_, errno,
              strerror(errno));
    // ROS_ERROR_STREAM(format);
  } else {
    log_format("video device format", format);
  }
  return rc;
}

int VideoDevice::set_format(const v4l2_format &format) {
  log_format("video device format", format);
  int rc = ioctl(fd_, VIDIOC_S_FMT, &format);
  if (rc == -1) {
    ROS_ERROR("ioctl(%d, VIDIOC_S_FMT) failed - errno=%d, %s", fd_, errno,
              strerror(errno));
  } else {
    log_format("set video device format", format);
  }
  return rc;
}

ssize_t VideoDevice::write(const unsigned char *buffer, size_t size) {
  ssize_t written = ::write(fd_, buffer, size);
  if (written != size) {
    ROS_WARN("write(%d, %zu) == %zu", fd_, size, written);
  }
  return written;
}

void log_format(const char *title, const v4l2_format &format) {
  ROS_INFO("%s:\n"
           "  type                 = %d\n"
           "  fmt.pix.width        = %d\n"
           "  fmt.pix.height       = %d\n"
           "  fmt.pix.pixelformat  = 0x%08X\n"
           "  fmt.pix.sizeimage    = %d\n"
           "  fmt.pix.field        = %d\n"
           "  fmt.pix.bytesperline = %d\n"
           "  fmt.pix.colorspace   = %d",
           title, format.type, format.fmt.pix.width, format.fmt.pix.height,
           format.fmt.pix.pixelformat, format.fmt.pix.sizeimage,
           format.fmt.pix.field, format.fmt.pix.bytesperline,
           format.fmt.pix.colorspace);
}
