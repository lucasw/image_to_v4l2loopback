/**
 * Copyright (c) 2013, Zhiwei Chu
 * Copyright (c) 2015, mayfieldrobotics.
 */

#ifndef IMAGE_TO_V4L2LOOPBACK_VIDEO_DEVICE_H
#define IMAGE_TO_V4L2LOOPBACK_VIDEO_DEVICE_H

#include <linux/videodev2.h>
#include <string>
#include <unistd.h>

/**
 * Represents a video capture device.
 */
class VideoDevice {
public:
  /**
   * \path Path to video capture device.
   */
  explicit VideoDevice(const std::string &path);

  VideoDevice(const VideoDevice &other);

  ~VideoDevice();

  int stream_on();

  int stream_off();

  int capabilities(v4l2_capability &capability);

  int get_format(v4l2_format &format);

  int set_format(const v4l2_format &format);

  ssize_t write(const unsigned char *buffer, size_t size);

private:
  void _log_format(const char *title, const v4l2_format &format);

  int _fd;
};

#endif  // IMAGE_TO_V4L2LOOPBACK_VIDEO_DEVICE_H
