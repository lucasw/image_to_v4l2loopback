#include "image_converter.h"

#include <gtest/gtest.h>
#include <linux/videodev2.h>

TEST(TestSuite, testImageConverterFormat) {
    ImageConverter ic(640, 480, "YV12");
    struct v4l2_format format = ic.format();
    ASSERT_EQ(format.type, V4L2_BUF_TYPE_VIDEO_OUTPUT);
    ASSERT_EQ(format.fmt.pix.field, V4L2_FIELD_NONE);
    ASSERT_EQ(format.fmt.pix.colorspace, V4L2_COLORSPACE_SRGB);
    ASSERT_EQ(format.fmt.pix.width, 640);
    ASSERT_EQ(format.fmt.pix.height, 480);
    ASSERT_EQ(format.fmt.pix.pixelformat, V4L2_PIX_FMT_YVU420);
    ASSERT_EQ(format.fmt.pix.bytesperline, 0);
    ASSERT_EQ(format.fmt.pix.sizeimage, 460800);
}

TEST(TestSuite, testImageConverterUnsupported) {
    ASSERT_THROW(ImageConverter(640, 480, "YUVP"), std::invalid_argument);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
