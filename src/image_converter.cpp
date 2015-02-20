#include "image_converter.h"

#include <string.h>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#define ROUND_UP_2(n) (((n) + 1) & ~1)

bool ImageConverter::is_supported(uint32_t fourcc) {
    return (
        fourcc == V4L2_PIX_FMT_BGR24 ||
        fourcc == V4L2_PIX_FMT_RGB24 ||
        fourcc == V4L2_PIX_FMT_GREY ||
        fourcc == V4L2_PIX_FMT_YVU420 ||
        fourcc == V4L2_PIX_FMT_YUYV
    );
}

bool ImageConverter::is_supported(const std::string& fourcc) {
    return fourcc.size() <= 4 && is_supported(_fourcc_code(fourcc));
}

ImageConverter::ImageConverter(
    uint32_t width,
    uint32_t height,
    const std::string& fourcc
    ) :
    _width(width),
    _height(height),
    _fourcc(_fourcc_code(fourcc)) {
    switch (_fourcc) {
        case V4L2_PIX_FMT_BGR24:
            _param_bgr24();
            break;
        case V4L2_PIX_FMT_RGB24:
            _param_rgb24();
            break;
        case V4L2_PIX_FMT_GREY:
            _param_grey();
            break;
        case V4L2_PIX_FMT_YVU420:
            _param_yvu420();
            break;
        case V4L2_PIX_FMT_YUYV:
            _param_yuyv();
            break;
        default:
            std::stringstream ss;
            ss << "Unsupported fourcc=" << _fourcc << ".";
            throw std::invalid_argument(ss.str());
    };
}

v4l2_format ImageConverter::format() const {
    v4l2_format format;
    format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    format.fmt.pix.field = V4L2_FIELD_NONE;
    format.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
    format.fmt.pix.width = _width;
    format.fmt.pix.height = _height;
    format.fmt.pix.pixelformat = _fourcc;
    format.fmt.pix.bytesperline = _bytes_per_line;
    format.fmt.pix.sizeimage = _size;
    return format;
}

bool ImageConverter::convert(const sensor_msgs::ImageConstPtr& msg, ImageConverter::Buffer& buffer) {
    // as opencv
    cv_bridge::CvImagePtr cv_msg = cv_bridge::toCvCopy(msg, _cv_copy_encoding);
    if (cv_msg == NULL) {
        ROS_INFO("failed to copy sensor image '%s' to cv image '%s'", msg->encoding.c_str(), _cv_copy_encoding.c_str());
        return false;
    }
    cv::Mat cv_image;
    cv::swap(cv_msg->image, cv_image);

    // resize
    if (msg->width  != _width || msg->height != _height) {
        cv::Mat cv_resize_image;
        cv::resize(cv_image, cv_resize_image, cv::Size(_width, _height));
        cv::swap(cv_image, cv_resize_image);
    }

    // convert color space
    if (_cv_color) {
        cv::Mat cv_cvt_image;
        cv::cvtColor(cv_image, cv_cvt_image, _cv_color_code, _cv_color_channels);
        cv::swap(cv_image, cv_cvt_image);
    }

    // format
    buffer.resize(_size);
    ((*this).*_fmt)(cv_image, buffer);

    return true;
}

bool ImageConverter::operator()(const sensor_msgs::ImageConstPtr& msg, ImageConverter::Buffer& buffer) {
    return convert(msg, buffer);
}

uint32_t ImageConverter::_fourcc_code(const std::string& fourcc) {
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

void ImageConverter::_param_bgr24() {
    _cv_copy_encoding = sensor_msgs::image_encodings::BGR8;
    _cv_color = false;
    _bytes_per_line = 0;
    _size = _width * _height * 3;
    _fmt = &ImageConverter::_fmt_bgr24;
}

void ImageConverter::_fmt_bgr24(const cv::Mat& image, Buffer& buf) {
    Buffer::value_type* b = &buf[0];
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

void ImageConverter::_param_rgb24() {
    _cv_copy_encoding = sensor_msgs::image_encodings::RGB8;
    _cv_color = false;
    _bytes_per_line = 0;
    _size = _width * _height * 3;
    _fmt = &ImageConverter::_fmt_rgb24;
}

void ImageConverter::_fmt_rgb24(const cv::Mat& image, Buffer& buf) {
    Buffer::value_type* b = &buf[0];
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

void ImageConverter::_param_grey() {
    _cv_copy_encoding = sensor_msgs::image_encodings::MONO8;
    _cv_color = false;
    _bytes_per_line = 0;
    _size = _width * _height;
    _fmt = &ImageConverter::_fmt_grey;
}

void ImageConverter::_fmt_grey(const cv::Mat& image, Buffer& buf) {
    Buffer::value_type* b = &buf[0];
    for (int row = 0; row != image.rows; row += 1) {
        for (int col = 0; col != image.cols; col += 1) {
            *(b++) = image.at<Buffer::value_type>(row, col);
        }
    }
}

void ImageConverter::_param_yvu420() {
    _cv_copy_encoding = sensor_msgs::image_encodings::BGR8;
    _cv_color = true;
    _cv_color_code = CV_BGR2YCrCb;
    _cv_color_channels = 0;
    _bytes_per_line = 0;
    _size = (_width * _height) + 2 * (ROUND_UP_2(_width) / 2 * ROUND_UP_2(_height) /2);
    _fmt = &ImageConverter::_fmt_yvu420;
}

void ImageConverter::_fmt_yvu420(const cv::Mat& image, Buffer& buf) {
    Buffer::value_type* b = &buf[0];
    Buffer::value_type* y = b;
    Buffer::value_type* cr = y + (_width * _height);
    Buffer::value_type* cb = cr + (ROUND_UP_2(_width) / 2 * ROUND_UP_2(_height) / 2);
    cv::Vec3b p;
    for (int row = 0; row != image.rows; row += 1) {
        for (int col = 0; col != image.cols; col += 1) {
            p = image.at<cv::Vec3b>(row, col);
            *(y++) = p[0];
            if (col % 2 == 0 ) {
                if (row % 2 == 0) {
                    *(cr++) = p[1];
                    *(cb++) = p[2];
                }
            }
        }
    }
}

void ImageConverter::_param_yuyv() {
    _cv_copy_encoding = sensor_msgs::image_encodings::BGR8;
    _cv_color = true;
    _cv_color_code = CV_BGR2YCrCb;
    _cv_color_channels = 0;
    _bytes_per_line = 0;
    _size = (_width * _height) * 2;
    _fmt = &ImageConverter::_fmt_yuyv;
}

void ImageConverter::_fmt_yuyv(const cv::Mat& image, Buffer& buf) {
    Buffer::value_type* b = &buf[0];
    cv::Vec3b p;
    for (int row = 0; row != image.rows; row += 1) {
        for (int col = 0; col != image.cols; col += 1) {
            p = image.at<cv::Vec3b>(row, col);
            *(b++) = p[0];
            *(b++) = (col % 2 == 0) ? p[2] : p[1];
        }
    }
}
