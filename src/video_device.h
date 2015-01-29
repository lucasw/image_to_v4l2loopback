#ifndef VIRTUAL_CAMERA_DEVICE_H_
#define VIRTUAL_CAMERA_DEVICE_H_

#include <string>

#include <linux/videodev2.h>
#include <unistd.h>

class VideoDevice {

public:

    VideoDevice(const std::string& path);

    VideoDevice(const VideoDevice& other);

    ~VideoDevice();

    int capabilities(const v4l2_capability& capability);

    int get_format(v4l2_format& format);

    int set_format(const v4l2_format& format);

    ssize_t write(const unsigned char* buffer, size_t size);

private:

    void _log_format(const char* title, const v4l2_format& format);

    int _fd;

};

#endif /* VIRTUAL_CAMERA_DEVICE_H_ */
