#include "command_line.h"

#include <linux/videodev2.h>

// Command line

CommandLine::CommandLine() :
    _video_device(
        "device",
        "Path to video device.",
        true,
        "",
        "PATH"
    ),
    _video_size(
        "s",
        "size",
        "Video size/dimensions in pixels.",
        false,
        Size(640, 480),
        "WIDTHxHEIGHT"
    ),
    _video_fourcc(
        "f",
        "fourcc",
        "Video FourCC.",
        false,
        "YV12",
        &_video_fourccs
    ),
    _queue_size(
        "q",
        "queue",
        "Stream queue size.",
        false,
        1,
        "NUMBER"
    ),
    _tclap("stream") {
    _tclap.add(_video_device);
    _tclap.add(_video_size);
    _tclap.add(_video_fourcc);
    _tclap.add(_queue_size);
}

void CommandLine::operator() (const std::vector<std::string>& args) {
    std::vector<std::string> t(args);
    _tclap.parse(t);
}

std::string CommandLine::video_device() {
    return _video_device.getValue();
}

CommandLine::Size CommandLine::video_size() {
    return _video_size.getValue();
}

std::string CommandLine::video_fourcc() {
    return _video_fourcc.getValue();
}

size_t CommandLine::queue_size() {
    return _queue_size.getValue();
}

TCLAP::ValuesConstraint<std::string> CommandLine::_init_video_fourccs() {
    std::vector<std::string> values;
    values.push_back("BGR3");
    values.push_back("RGB3");
    values.push_back("GREY");
    values.push_back("YV12");
    values.push_back("YUYV");
    return TCLAP::ValuesConstraint<std::string>(values);
}

TCLAP::ValuesConstraint<std::string> CommandLine::_video_fourccs = CommandLine::_init_video_fourccs();

// CommandLine::Size

CommandLine::Size::Size(size_t width_, size_t height_) :
    width(width_),
    height(height_) {
}

std::ostream& operator<< (std::ostream& os, const CommandLine::Size& v) {
   os << v.width << "x" << v.height;
   return os;
}

std::istream& operator>> (std::istream& is, CommandLine::Size& v) {
    std::istreambuf_iterator<char> i(is), e;
    std::string b;

    // width
    b.clear();
    while (i != e) {
        if (!std::isdigit(*i)) {
            break;
        }
        b += *i;
        i++;
    }
    if (b.empty()) {
        is.setstate(std::istream::failbit);
        return is;
    }
    v.width = atoi(b.c_str());

    // x
    if (*i != 'x' and *i != 'X') {
        is.setstate(std::istream::failbit);
        return is;
    }
    i++;

    // height
    b.clear();
    while (i != e) {
        if (!std::isdigit(*i)) {
            break;
        }
        b += *i;
        i++;
    }
    if (b.empty()) {
        is.setstate(std::istream::failbit);
        return is;
    }
    v.height = atoi(b.c_str());

    if  (i != e) {
        is.setstate(std::istream::failbit);
        return is;
    }

    return is;
}
