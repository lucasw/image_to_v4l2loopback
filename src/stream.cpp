/**
 * Based on https://github.com/czw90130/virtual_camera/. It will:
 *
 * - subscribe to a sensor_msgs::Image topic
 * - feed received images to an ImageConverter.
 * - write the converted image to a VideoDevice.
 *
 * \copyright Copyright (c) 2013, Zhiwei Chu
 * \copyright Copyright (c) 2015, mayfieldrobotics.
 * \license This project is released under the BSD License.
 *
 */

#include <image_to_v4l2loopback/image_converter.h>
#include <image_to_v4l2loopback/video_device.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <string>

/**
 * Writes converted images from a sensor_msgs::Image to a VideoDevice.
 */
class ImageStream {
public:
  ImageStream(const std::string &topic, ImageConverter &image_converter, VideoDevice &dev,
              size_t queue_size):
    _dev(dev),
    _image_converter(image_converter),
    _transport(ros::NodeHandle()),
    _sub(_transport.subscribe(topic, queue_size, &ImageStream::stream, this))
  {
  }

  /// Converts sensor_msgs::Image and writes resulting image data to a
  /// VideoDevice.
  void stream(const sensor_msgs::ImageConstPtr &msg) {
    int rc = _image_converter(msg, _buf);
    if (rc == -1) {
      return;
    }
    rc = _dev.write(&_buf[0], _buf.size());
    if (rc == -1) {
      return;
    }
  }

  void operator()(const sensor_msgs::ImageConstPtr &msg) { stream(msg); }

private:
  ImageConverter _image_converter;

  ImageConverter::Buffer _buf;

  VideoDevice _dev;

  image_transport::ImageTransport _transport;

  image_transport::Subscriber _sub;
};

// TODO(lucasw) probably will merge this with ImageStream later,
// for now convenient to have it be another layer.
class ImageStreamNode {
public:
  ImageStreamNode()
  {
    int width_tmp = 640;
    ros::param::get("~width", width_tmp);
    int height_tmp = 480;
    ros::param::get("~height", height_tmp);

    const uint32_t width = width_tmp;
    const uint32_t height = height_tmp;

    // valid options are BGR3, RGB3, GREY, YV12, YUYV
    std::string format = "YV12";
    ros::param::get("~format", format);
    ROS_INFO("converting - width=%u, height=%u, format=%s", width, height,
             format.c_str());
    ImageConverter image_converter(width, height, format);

    std::string video_device = "/dev/video1";
    ros::param::get("~device", video_device);

    ROS_INFO("opening '%s'", video_device.c_str());
    VideoDevice dev(video_device);

    int rc;

    v4l2_capability capability;
    rc = dev.capabilities(capability);
    if (rc == -1) {
      throw std::runtime_error("failed device caps");
    }
    ROS_INFO("'%s' caps %#08x", video_device.c_str(), capability.capabilities);

    v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    rc = dev.get_format(fmt);
    if (rc == -1) {
      throw std::runtime_error("failed to get device format");
    }
    log_format("current format: ", fmt);

    ROS_INFO("setting '%s' format", video_device.c_str());
    fmt = image_converter.format();
    rc = dev.set_format(fmt);
    if (rc == -1) {
      throw std::runtime_error("failed device format: " + format);
    }

    ROS_INFO("turn on '%s' streaming", video_device.c_str());
    rc = dev.stream_on();
    if (rc == -1) {
      throw std::runtime_error("failed device stream");
    }
    int queue_size = 1;
    ros::param::get("~queue_size", queue_size);
    ROS_INFO("streaming images from '%s' to '%s' w/ queue-size=%u",
             ros::names::resolve("image", true).c_str(), video_device.c_str(),
             queue_size);
    image_stream_ = std::make_unique<ImageStream>("image", image_converter, dev, queue_size);
  }

private:
  std::unique_ptr<ImageStream> image_stream_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "stream", ros::init_options::AnonymousName);
  ImageStreamNode image_stream_node;
  ros::spin();

  return 0;
}
