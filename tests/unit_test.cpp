#include "image_converter.h"
#include "video_device.h"

#include <gtest/gtest.h>
#include <linux/videodev2.h>

// ImageConverter

TEST(TestSuite, testImageConverterFormat) {
    struct Format {
        std::string fourcc_str;
        int fourcc;
        size_t size;
    };
    Format formats[] = {
        {"BGR3", V4L2_PIX_FMT_BGR24, 921600},
        {"RGB3", V4L2_PIX_FMT_RGB24, 921600},
        {"GREY", V4L2_PIX_FMT_GREY, 307200},
        {"YV12", V4L2_PIX_FMT_YVU420, 460800},
        {"YUYV", V4L2_PIX_FMT_YUYV, 614400}
    };

    for (size_t i = 0 ; i < sizeof formats /sizeof formats[0]; i++) {
        ImageConverter ic(640, 480, formats[i].fourcc_str);
        struct v4l2_format format = ic.format();
        ASSERT_EQ(V4L2_BUF_TYPE_VIDEO_OUTPUT, format.type);
        ASSERT_EQ(V4L2_FIELD_NONE, format.fmt.pix.field);
        ASSERT_EQ(V4L2_COLORSPACE_SRGB, format.fmt.pix.colorspace);
        ASSERT_EQ(640, format.fmt.pix.width);
        ASSERT_EQ(480, format.fmt.pix.height);
        ASSERT_EQ(formats[i].fourcc, format.fmt.pix.pixelformat);
        ASSERT_EQ(0, format.fmt.pix.bytesperline);
        ASSERT_EQ(formats[i].size, format.fmt.pix.sizeimage);
    }
}

TEST(TestSuite, testImageConverterUnsupported) {
    ASSERT_THROW(ImageConverter(640, 480, "YUVP"), std::invalid_argument);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
