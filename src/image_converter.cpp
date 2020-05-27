/**
 * Copyright (c) 2013, Zhiwei Chu
 * Copyright (c) 2015, mayfieldrobotics.
 */

#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <image_to_v4l2loopback/image_converter.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <string.h>
#include <string>
#include <utility>  // TODO(lucasw) roslint want this for cv::swap

#define ROUND_UP_2(n) (((n) + 1) & ~1)

bool ImageConverter::is_supported(uint32_t fourcc) {
  return (fourcc == V4L2_PIX_FMT_BGR24 || fourcc == V4L2_PIX_FMT_RGB24 ||
          fourcc == V4L2_PIX_FMT_GREY || fourcc == V4L2_PIX_FMT_YVU420 ||
          fourcc == V4L2_PIX_FMT_YUYV);
}

bool ImageConverter::is_supported(const std::string &fourcc) {
  return fourcc.size() <= 4 && is_supported(_fourcc_code(fourcc));
}

ImageConverter::ImageConverter(uint32_t width, uint32_t height,
                               const std::string &fourcc) :
  width_(width),
  height_(height),
  fourcc_(_fourcc_code(fourcc))
{
  switch (fourcc_) {
  case V4L2_PIX_FMT_BGR24:
    param_bgr24();
    break;
  case V4L2_PIX_FMT_RGB24:
    param_rgb24();
    break;
  case V4L2_PIX_FMT_GREY:
    param_grey();
    break;
  case V4L2_PIX_FMT_YVU420:
    param_yvu420();
    break;
  case V4L2_PIX_FMT_YUYV:
    param_yuyv();
    break;
  default:
    std::stringstream ss;
    ss << "Unsupported fourcc=" << fourcc_ << ".";
    throw std::invalid_argument(ss.str());
  }
}

v4l2_format ImageConverter::format() const {
  v4l2_format format;
  format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  format.fmt.pix.field = V4L2_FIELD_NONE;
  format.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
  format.fmt.pix.width = width_;
  format.fmt.pix.height = height_;
  format.fmt.pix.pixelformat = fourcc_;
  format.fmt.pix.bytesperline = bytes_per_line_;
  format.fmt.pix.sizeimage = size_;
  return format;
}

bool ImageConverter::convert(const sensor_msgs::ImageConstPtr &msg,
                             ImageConverter::Buffer &buffer) {
  // TODO(lucasw) it would be nice to avoid the resize if possible,
  // crop if necessary
  // as opencv
  cv_bridge::CvImagePtr cv_msg = cv_bridge::toCvCopy(msg, cv_copy_encoding_);
  if (cv_msg == NULL) {
    ROS_INFO("failed to copy sensor image '%s' to cv image '%s'",
             msg->encoding.c_str(), cv_copy_encoding_.c_str());
    return false;
  }
  cv::Mat cv_image;
  cv::swap(cv_msg->image, cv_image);

  // resize
  if (msg->width != width_ || msg->height != height_) {
    cv::Mat cv_resize_image;
    cv::resize(cv_image, cv_resize_image, cv::Size(width_, height_));
    cv::swap(cv_image, cv_resize_image);
  }

  // convert color space
  if (cv_color_) {
    cv::Mat cv_cvt_image;
    cv::cvtColor(cv_image, cv_cvt_image, cv_color_code_, cv_color_channels_);
    cv::swap(cv_image, cv_cvt_image);
  }

  // format
  buffer.resize(size_);
  ((*this).*fmt_)(cv_image, buffer);

  return true;
}

bool ImageConverter::operator()(const sensor_msgs::ImageConstPtr &msg,
                                ImageConverter::Buffer &buffer) {
  return convert(msg, buffer);
}

uint32_t ImageConverter::_fourcc_code(const std::string &fourcc) {
  if (fourcc.size() > 4) {
    std::stringstream ss;
    ss << "Invalid fourcc='" << fourcc << "'.";
    throw std::runtime_error(ss.str());
  }
  std::stringstream ss;
  ss << std::setw(4) << fourcc;
  std::string p = ss.str();
  return v4l2_fourcc(p[0], p[1], p[2], p[3]);
}

void ImageConverter::param_bgr24() {
  ROS_INFO_STREAM("bgr24");
  cv_copy_encoding_ = sensor_msgs::image_encodings::BGR8;
  cv_color_ = false;
  bytes_per_line_ = 0;
  size_ = width_ * height_ * 3;
  fmt_ = &ImageConverter::fmt_bgr24;
}

void ImageConverter::fmt_bgr24(const cv::Mat &image, Buffer &buf) {
  Buffer::value_type *b = &buf[0];
  cv::Vec3b p;
  for (int row = 0; row != image.rows; row += 1) {
    for (int col = 0; col != image.cols; col += 1) {
      p = image.at<cv::Vec3b>(row, col);
      *(b++) = p[0];
      *(b++) = p[1];
      *(b++) = p[2];
    }
  }
}

void ImageConverter::param_rgb24() {
  ROS_INFO_STREAM("rgb24");
  cv_copy_encoding_ = sensor_msgs::image_encodings::BGR8;
  cv_copy_encoding_ = sensor_msgs::image_encodings::RGB8;
  cv_color_ = false;
  bytes_per_line_ = 0;
  size_ = width_ * height_ * 3;
  fmt_ = &ImageConverter::fmt_rgb24;
}

void ImageConverter::fmt_rgb24(const cv::Mat &image, Buffer &buf) {
  Buffer::value_type *b = &buf[0];
  cv::Vec3b p;
  for (int row = 0; row != image.rows; row += 1) {
    for (int col = 0; col != image.cols; col += 1) {
      p = image.at<cv::Vec3b>(row, col);
      *(b++) = p[0];
      *(b++) = p[1];
      *(b++) = p[2];
    }
  }
}

void ImageConverter::param_grey() {
  ROS_INFO_STREAM("grey");
  cv_copy_encoding_ = sensor_msgs::image_encodings::MONO8;
  cv_color_ = false;
  bytes_per_line_ = 0;
  size_ = width_ * height_;
  fmt_ = &ImageConverter::fmt_grey;
}

void ImageConverter::fmt_grey(const cv::Mat &image, Buffer &buf) {
  Buffer::value_type *b = &buf[0];
  for (int row = 0; row != image.rows; row += 1) {
    for (int col = 0; col != image.cols; col += 1) {
      *(b++) = image.at<Buffer::value_type>(row, col);
    }
  }
}

void ImageConverter::param_yvu420() {
  ROS_INFO_STREAM("yvu420");
  cv_copy_encoding_ = sensor_msgs::image_encodings::BGR8;
  cv_color_ = true;
  cv_color_code_ = CV_BGR2YCrCb;
  cv_color_channels_ = 0;
  bytes_per_line_ = 0;
  size_ = (width_ * height_) +
          2 * (ROUND_UP_2(width_) / 2 * ROUND_UP_2(height_) / 2);
  fmt_ = &ImageConverter::fmt_yvu420;
}

void ImageConverter::fmt_yvu420(const cv::Mat &image, Buffer &buf) {
  Buffer::value_type *b = &buf[0];
  Buffer::value_type *y = b;
  Buffer::value_type *cr = y + (width_ * height_);
  Buffer::value_type *cb =
      cr + (ROUND_UP_2(width_) / 2 * ROUND_UP_2(height_) / 2);
  cv::Vec3b p;
  for (int row = 0; row != image.rows; row += 1) {
    for (int col = 0; col != image.cols; col += 1) {
      p = image.at<cv::Vec3b>(row, col);
      *(y++) = p[0];
      if (col % 2 == 0) {
        if (row % 2 == 0) {
          *(cr++) = p[1];
          *(cb++) = p[2];
        }
      }
    }
  }
}

void ImageConverter::param_yuyv() {
  ROS_INFO_STREAM("yuyv");
  cv_copy_encoding_ = sensor_msgs::image_encodings::BGR8;
  cv_color_ = true;
  cv_color_code_ = CV_BGR2YCrCb;
  cv_color_channels_ = 0;
  bytes_per_line_ = 0;
  size_ = (width_ * height_) * 2;
  fmt_ = &ImageConverter::fmt_yuyv;
}

void ImageConverter::fmt_yuyv(const cv::Mat &image, Buffer &buf) {
  Buffer::value_type *b = &buf[0];
  cv::Vec3b p;
  for (int row = 0; row != image.rows; row += 1) {
    for (int col = 0; col != image.cols; col += 1) {
      p = image.at<cv::Vec3b>(row, col);
      *(b++) = p[0];
      *(b++) = (col % 2 == 0) ? p[2] : p[1];
    }
  }
}
