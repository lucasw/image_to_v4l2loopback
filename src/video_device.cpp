#include "video_device.h"

#include <stdexcept>

#include <fcntl.h>
#include <ros/ros.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>


VideoDevice::VideoDevice(const std::string& path) : _fd(-1) {
    _fd = open(path.c_str(), O_RDWR);
    if (_fd == -1) {
        std::stringstream ss;
        ss << "open('" << path << "') failed - errno=" << errno << " ('" << strerror(errno) << "')";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
}

VideoDevice::VideoDevice(const VideoDevice& other) : _fd(-1) {
    _fd = dup(other._fd);
    if (_fd == -1) {
        std::stringstream ss;
        ss << "dup('" << other._fd << "') failed - errno=" << errno << " ('" << strerror(errno) << "')";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
}

VideoDevice::~VideoDevice() {
    close(_fd);
    _fd = -1;
}

int VideoDevice::stream_on() {
    int type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    int rc = ioctl(_fd, VIDIOC_STREAMON, &type);
    if (rc == -1) {
        ROS_ERROR("ioctl(%d, VIDIOC_STREAMON) failed - errno=%d, %s", _fd, errno, strerror(errno));
    }
    return rc;
}

int VideoDevice::stream_off() {
    int type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    int rc = ioctl(_fd, VIDIOC_STREAMOFF, &type);
    if (rc == -1) {
        ROS_ERROR("ioctl(%d, VIDIOC_STREAMOFF) failed - errno=%d, %s", _fd, errno, strerror(errno));
    }
    return rc;
}

int VideoDevice::capabilities(v4l2_capability& capability) {
    int rc = ioctl(_fd, VIDIOC_QUERYCAP, &capability);
    if (rc == -1) {
        ROS_ERROR("ioctl(%d, VIDIOC_QUERYCAP) failed - errno=%d, %s", _fd, errno, strerror(errno));
    }
    return rc;
}

int VideoDevice::get_format(v4l2_format& format) {
    int rc = ioctl(_fd, VIDIOC_G_FMT, &format);
    if (rc == -1) {
        ROS_ERROR("ioctl(%d, VIDIOC_G_FMT) failed - errno=%d, %s", _fd, errno, strerror(errno));
    } else {
        _log_format("video device format", format);
    }
    return rc;
}

int VideoDevice::set_format(const v4l2_format& format) {
    int rc= ioctl(_fd, VIDIOC_S_FMT, &format);
    if (rc == -1) {
        ROS_ERROR("ioctl(%d, VIDIOC_S_FMT) failed - errno=%d, %s", _fd, errno, strerror(errno));
    } else {
        _log_format("video device format", format);
    }
    return rc;
}

ssize_t VideoDevice::write(const unsigned char* buffer, size_t size) {
    ssize_t written = ::write(_fd, buffer, size);
    if (written != size) {
        ROS_WARN("write(%d, %zu) == %zu", _fd, size, written);
    }
    return written;
}

void VideoDevice::_log_format(const char* title, const v4l2_format& format) {
    ROS_INFO(
        "%s:\n"
        "  type                =%d\n"
        "  fmt.pix.width       =%d\n"
        "  fmt.pix.height      =%d\n"
        "  fmt.pix.pixelformat =%d\n"
        "  fmt.pix.sizeimage   =%d\n"
        "  fmt.pix.field       =%d\n"
        "  fmt.pix.bytesperline=%d\n"
        "  fmt.pix.colorspace  =%d",
        title,
        format.type,
        format.fmt.pix.width,
        format.fmt.pix.height,
        format.fmt.pix.pixelformat,
        format.fmt.pix.sizeimage,
        format.fmt.pix.field,
        format.fmt.pix.bytesperline,
        format.fmt.pix.colorspace
    );
}
