#ifndef VIRTUAL_CAM_COMMAND_LINE_H_
#define VIRTUAL_CAM_COMMAND_LINE_H_

#include "tclap/CmdLine.h"

/**
 * Parses command-lines arguments.
 */
class CommandLine {

public:

    CommandLine();

    CommandLine(const std::vector<std::string>& args);

    /**
     * Parses command line arguments.
     * \args Array of command line argument to parse.
     * \throws
     */
    void parse(const std::vector<std::string>& args);

    /// Alias for CommandLine::parse.
    void operator()(const std::vector<std::string>& args);

    struct Size {

        Size(size_t width_, size_t height_);

        size_t width;

        size_t height;

    };

    /// Path to video capture device. Image data is written to this.
    std::string video_device();

    /// Size of the image to write to CommandLine::video_device.
    Size video_size();

    /// Convert image to this format before writing to CommandLine::video_device.
    std::string video_fourcc();

    /// Size of incoming sensor_msgs/Image topic queue.
    size_t queue_size();

private:

    static TCLAP::ValuesConstraint<std::string> _video_fourccs;

    static TCLAP::ValuesConstraint<std::string> _init_video_fourccs();

    TCLAP::UnlabeledValueArg<std::string> _video_device;

    TCLAP::ValueArg<Size> _video_size;

    TCLAP::ValueArg<std::string> _video_fourcc;

    TCLAP::ValueArg<unsigned int> _queue_size;

    TCLAP::CmdLine _tclap;

};

std::ostream& operator<< (std::ostream& os, const CommandLine::Size& v);

std::istream& operator>> (std::istream& is, CommandLine::Size& v);

namespace TCLAP {

    template <>
    struct ArgTraits<CommandLine::Size> {

        typedef ValueLike ValueCategory;

    };

}

#endif /* VIRTUAL_CAM_COMMAND_LINE_H_ */
