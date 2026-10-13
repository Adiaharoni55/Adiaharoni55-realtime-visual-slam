#pragma once

#include "rtvm/pose.hpp"
#include <string>
#include <vector>

namespace rtvm {

// One line of rgb.txt or depth.txt: a timestamp and an image path.
struct TimedFile {
    double timestamp;   // seconds
    std::string path;   // relative to the sequence folder, e.g. "rgb/1305031102.175304.png"
};

// Read rgb.txt or depth.txt. Skips comment lines (#). Throws if the file can't be opened.
std::vector<TimedFile> readTumFileList(const std::string& path);

// One line of groundtruth.txt: a timestamp and the true camera pose.
struct TimedPose {
    double timestamp;   // seconds
    Pose pose;          // camera pose in the world (motion-capture) frame
};

// Read groundtruth.txt ("timestamp tx ty tz qx qy qz qw"). Skips comment lines (#).
// Throws if the file can't be opened or a line is malformed.
std::vector<TimedPose> readTumGroundTruth(const std::string& path);

}  // namespace rtvm