#include "rtvm/tum_dataset.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace rtvm {

std::vector<TimedFile> readTumFileList(const std::string& path) {
    // Open the list file (rgb.txt or depth.txt).
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("readTumFileList: cannot open " + path);
    }

    // Each data line is "<timestamp> <image path>". Timestamps are needed later
    // to match each RGB image with its depth image and ground-truth pose.
    std::vector<TimedFile> files;
    std::string line;
    while (std::getline(file, line)) {
        // Skip empty lines and comment lines.
        if (line.empty() || line[0] == '#') {
            continue;
        }

        // Split the line into its two fields; fail loudly on a malformed line.
        std::istringstream ss(line);
        double timestamp;
        std::string name;
        if (!(ss >> timestamp >> name)) {
            throw std::runtime_error("readTumFileList: bad line in " + path + ": " + line);
        }

        files.push_back({timestamp, name});
    }
    return files;
}

std::vector<TimedPose> readTumGroundTruth(const std::string& path){
    // Open the groundtruth file ("timestamp tx ty tz qx qy qz qw")
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("readTumGroundTruth: cannot open " + path);
    }

    // Each data line is "<timestamp> tx ty tz qx qy qz qw" (a pose from motion capture)
    std::vector<TimedPose> poses;
    std::string line;
    while (std::getline(file, line)) {
        // Skip empty lines and comment lines.
        if (line.empty() || line[0] == '#') {
            continue;
        }

        // Split the line into its 8 fields; fail loudly on a malformed line.
        std::istringstream ss(line);
        double timestamp, tx, ty, tz, qx, qy, qz, qw;
        if (!(ss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
            throw std::runtime_error("readTumGroundTruth: bad line in " + path + ": " + line);
        }

        poses.push_back({timestamp, poseFromTum(tx, ty, tz, qx, qy, qz, qw)});
    }
    return poses;   
}

}  // namespace rtvm