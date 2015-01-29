#ifndef VIRTUAL_CAMERA_COMMAND_LINE_H_
#define VIRTUAL_CAMERA_COMMAND_LINE_H_

#include "tclap/CmdLine.h"

class CommandLine {

public:

    CommandLine();

    void operator()(const std::vector<std::string>& args);

    struct Size {

        Size(size_t width_, size_t height_);

        size_t width;

        size_t height;

    };

    std::string video_device();

    Size video_size();

    std::string video_fourcc();

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

#endif /* VIRTUAL_CAMERA_COMMAND_LINE_H_ */
