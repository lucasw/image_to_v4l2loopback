/*
 * Copyright (c) 2013, Zhiwei Chu
 * Copyright (c) 2015, mayfieldrobotics.
 */

#ifndef IMAGE_TO_V4L2LOOPBACK_IMAGE_CONVERTER_H
#define IMAGE_TO_V4L2LOOPBACK_IMAGE_CONVERTER_H

#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <string>
#include <vector>

/**
 * Resizes and formats an image
 */
class ImageConverter {
public:
  // Determine whether a target pixel format is supported.
  static bool is_supported(uint32_t fourcc);
  static bool is_supported(const std::string &fourcc);

  /**
   * \width Target image width in pixels.
   * \height Target image height in pixels.
   * \fourcc Target image pixel format as a fourcc string.
   */
  ImageConverter(uint32_t width, uint32_t height, const std::string &fourcc);

  // The VideoDevice: to support writing images generated be this conveter.
  v4l2_format format() const;

  typedef std::vector<unsigned char> Buffer;

  bool convert(const sensor_msgs::ImageConstPtr &msg, Buffer &buf);

  // Alias for ImageConverter::convert.
  bool operator()(const sensor_msgs::ImageConstPtr &msg, Buffer &buf);

private:
  static uint32_t _fourcc_code(const std::string &fourcc);

  void param_bgr24();
  void fmt_bgr24(const cv::Mat &image, Buffer &buf);

  void param_rgb24();
  void fmt_rgb24(const cv::Mat &image, Buffer &buf);

  void param_grey();
  void fmt_grey(const cv::Mat &image, Buffer &buf);

  void param_y16();
  void fmt_y16(const cv::Mat &image, Buffer &buf);

  void param_yvu420();
  void fmt_yvu420(const cv::Mat &image, Buffer &buf);

  void param_yuyv();
  void fmt_yuyv(const cv::Mat &image, Buffer &buf);

  uint32_t width_;
  uint32_t height_;
  uint32_t fourcc_;
  std::string cv_copy_encoding_;
  bool cv_color_;
  int cv_color_code_;
  int cv_color_channels_;
  uint32_t bytes_per_line_;
  uint32_t size_;

  void (ImageConverter::*fmt_)(const cv::Mat &image, Buffer &buf);
};

#endif  // IMAGE_TO_V4L2LOOPBACK_IMAGE_CONVERTER_H
