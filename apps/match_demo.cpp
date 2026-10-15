#include <iostream>
#include <string>

#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>

#include "rtvm/features.hpp"
#include "rtvm/tum_dataset.hpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: rtvm_match_demo <path to TUM sequence folder>\n";
        return 1;
    }
    std::string dir = argv[1];

    // Load and associate the sequence (only the image paths are needed here).
    auto rgb = rtvm::readTumFileList(dir + "/rgb.txt");
    auto depth = rtvm::readTumFileList(dir + "/depth.txt");
    auto gt = rtvm::readTumGroundTruth(dir + "/groundtruth.txt");
    auto frames = rtvm::associateTum(rgb, depth, gt);
    std::cout << "Associated frames: " << frames.size() << '\n';

    // Select two frames
    auto frame1 = frames[0];
    auto frame2 = frames[10];

    // How far did the camera truly move between the two frames? (from ground truth)
    rtvm::Pose motion = rtvm::relativePose(frame1.ground_truth, frame2.ground_truth);
    std::cout << "True camera movement: " << motion.translation().norm() * 100 << " cm\n";
    std::cout << "True camera rotation: "
            << Eigen::AngleAxisd(motion.linear()).angle() * 180.0 / EIGEN_PI << " degrees\n";

    // Two frames a short time apart (10 frames = about 1/3 of a second).
    cv::Mat img_a = cv::imread(dir + "/" + frame1.rgb_path, cv::IMREAD_GRAYSCALE);
    cv::Mat img_b = cv::imread(dir + "/" + frame2.rgb_path, cv::IMREAD_GRAYSCALE);

    rtvm::FrameFeatures fa = rtvm::detectOrb(img_a);
    rtvm::FrameFeatures fb = rtvm::detectOrb(img_b);
    std::vector<cv::DMatch> matches = rtvm::matchFeatures(fa, fb);

    std::cout << "Keypoints: " << fa.keypoints.size() << " and " << fb.keypoints.size()
              << ", good matches: " << matches.size() << '\n';

    // Draw both images side by side with lines between matched keypoints.
    cv::Mat out;
    cv::drawMatches(img_a, fa.keypoints, img_b, fb.keypoints, matches, out);
    cv::imwrite("matches.png", out);
    std::cout << "Saved matches.png\n";
    return 0;
}