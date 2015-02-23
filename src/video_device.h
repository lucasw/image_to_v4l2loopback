#ifndef VIRTUAL_CAM_DEVICE_H_
#define VIRTUAL_CAM_DEVICE_H_

#include <string>

#include <linux/videodev2.h>
#include <unistd.h>

/**
 * Represents a video capture device.
 */
class VideoDevice {

public:

    /**
     * \path Path to video capture device.
     */
    VideoDevice(const std::string& path);

    VideoDevice(const VideoDevice& other);

    ~VideoDevice();

    int stream_on();

    int stream_off();

    int capabilities(v4l2_capability& capability);

    int get_format(v4l2_format& format);

    int set_format(const v4l2_format& format);

    ssize_t write(const unsigned char* buffer, size_t size);

private:

    void _log_format(const char* title, const v4l2_format& format);

    int _fd;

};

#endif /* VIRTUAL_CAM_DEVICE_H_ */
