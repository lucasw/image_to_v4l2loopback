#ifndef VIRTUAL_CAM_IMAGE_CONVERTER_H_
#define VIRTUAL_CAM_IMAGE_CONVERTER_H_

#include <linux/videodev2.h>
#include <opencv/cv.hpp>
#include <sensor_msgs/Image.h>

/**
 * Resizes and formats an image
 */
class ImageConverter {

public:

    /// Determines whether a target pixel format is supported.
    static bool is_supported(uint32_t fourcc);

    /// Determines whether a target pixel format is supported.
    static bool is_supported(const std::string& fourcc);

    /**
     * \width Target image width in pixels.
     * \height Target image height in pixels.
     * \fourcc Target image pixel format as a fourcc string.
     *
     */
    ImageConverter(uint32_t width, uint32_t height, const std::string& fourcc);

    /// The VideoDevice: to support writing images generated be this conveter.
    v4l2_format format() const;

    typedef std::vector<unsigned char> Buffer;

    bool convert(const sensor_msgs::ImageConstPtr& msg, Buffer& buf);

    /// Alias for ImageConverter::convert.
    bool operator()(const sensor_msgs::ImageConstPtr& msg, Buffer& buf);

private:

    static uint32_t _fourcc_code(const std::string& fourcc);

    void _param_bgr24();

    void _fmt_bgr24(const cv::Mat& image, Buffer& buf);

    void _param_rgb24();

    void _fmt_rgb24(const cv::Mat& image, Buffer& buf);

    void _param_grey();

    void _fmt_grey(const cv::Mat& image, Buffer& buf);

    void _param_y16();

    void _fmt_y16(const cv::Mat& image, Buffer& buf);

    void _param_yvu420();

    void _fmt_yvu420(const cv::Mat& image, Buffer& buf);

    void _param_yuyv();

    void _fmt_yuyv(const cv::Mat& image, Buffer& buf);

    uint32_t _width;

    uint32_t _height;

    uint32_t _fourcc;

    std::string _cv_copy_encoding;

    bool _cv_color;

    int _cv_color_code;

    int _cv_color_channels;

    uint32_t _bytes_per_line;

    uint32_t _size;

    void (ImageConverter::*_fmt)(const cv::Mat& image, Buffer& buf);

};

#endif /* VIRTUAL_CAM_IMAGE_CONVERTER_H_ */
