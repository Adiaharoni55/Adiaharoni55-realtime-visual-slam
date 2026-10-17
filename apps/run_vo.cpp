#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "rtvm/camera.hpp"
#include "rtvm/tum_dataset.hpp"
#include "rtvm/visual_odometry.hpp"

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: rtvm_run_vo <TUM sequence folder> <camera yaml>\n";
        return 1;
    }
    std::string dir = argv[1];
    rtvm::PinholeCamera camera = rtvm::loadCameraFromYaml(argv[2]);

    auto frames = rtvm::associateTum(rtvm::readTumFileList(dir + "/rgb.txt"),
                                     rtvm::readTumFileList(dir + "/depth.txt"),
                                     rtvm::readTumGroundTruth(dir + "/groundtruth.txt"));
    std::cout << "Frames: " << frames.size() << '\n';

    rtvm::VisualOdometry vo(camera);
    std::vector<rtvm::TimedPose> trajectory;

    auto start = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < frames.size(); ++i) {
        cv::Mat gray = cv::imread(dir + "/" + frames[i].rgb_path, cv::IMREAD_GRAYSCALE);
        cv::Mat depth = cv::imread(dir + "/" + frames[i].depth_path, cv::IMREAD_UNCHANGED);

        rtvm::Pose pose = vo.track(gray, depth);
        trajectory.push_back({frames[i].timestamp, pose});

        if (i % 100 == 0) {
            std::cout << "Frame " << i << " / " << frames.size() << '\n';
        }
    }
    auto end = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(end - start).count();

    std::filesystem::create_directories("output");
    rtvm::writeTumTrajectory("output/trajectory.txt", trajectory);

    std::cout << "Failed frames: " << vo.failures() << '\n'
              << "Average time per frame: " << ms / frames.size() << " ms\n"
              << "Saved output/trajectory.txt\n";
    return 0;
}