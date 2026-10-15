#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "rtvm/motion.hpp"

namespace {

// A synthetic two-frame scene with a known camera motion.
struct SyntheticScene {
    rtvm::PinholeCamera camera{500.0, 500.0, 320.0, 240.0};   // no distortion
    rtvm::Pose true_motion;          // camera B's pose seen from camera A
    rtvm::FrameFeatures a, b;        // keypoints only (no descriptors needed)
    std::vector<cv::DMatch> matches; // keypoint i in A <-> keypoint i in B
    cv::Mat depth_a;                 // 16-bit TUM-style depth image for A
};

SyntheticScene makeScene(const rtvm::Pose& motion) {
    SyntheticScene s;
    s.true_motion = motion;
    s.depth_a = cv::Mat(480, 640, CV_16UC1, cv::Scalar(0));

    // A 10 x 8 grid of 3D points in A's frame, 2-4 m in front of the camera.
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 8; ++j) {
            Eigen::Vector3d X_a(-0.9 + 0.2 * i, -0.6 + 0.17 * j, 2.0 + 0.25 * ((i + j) % 9));

            // The same point seen from camera B: undo B's pose.
            Eigen::Vector3d X_b = motion.inverse() * X_a;

            Eigen::Vector2d pa = s.camera.project(X_a);
            Eigen::Vector2d pb = s.camera.project(X_b);

            // Write the depth at pa's pixel (TUM format: 5000 = 1 m).
            int u = static_cast<int>(std::lround(pa.x()));
            int v = static_cast<int>(std::lround(pa.y()));
            s.depth_a.at<std::uint16_t>(v, u) = static_cast<std::uint16_t>(std::lround(X_a.z() * 5000.0));

            int index = static_cast<int>(s.a.keypoints.size());
            s.a.keypoints.emplace_back(static_cast<float>(pa.x()), static_cast<float>(pa.y()), 1.0f);
            s.b.keypoints.emplace_back(static_cast<float>(pb.x()), static_cast<float>(pb.y()), 1.0f);
            s.matches.emplace_back(index, index, 0.0f);
        }
    }
    return s;
}

// A motion of 10 cm right, 5 cm forward, turning 5 degrees around the vertical axis.
rtvm::Pose exampleMotion() {
    rtvm::Pose motion = rtvm::Pose::Identity();
    motion.linear() = Eigen::AngleAxisd(5.0 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    motion.translation() = Eigen::Vector3d(0.10, 0.0, 0.05);
    return motion;
}

}  // namespace

// With perfect data, estimateMotion recovers the exact motion, and every point is an inlier.
TEST(MotionTest, RecoversKnownMotion) {
    SyntheticScene s = makeScene(exampleMotion());

    rtvm::MotionEstimate est = rtvm::estimateMotion(s.a, s.b, s.matches, s.depth_a, s.camera);

    ASSERT_TRUE(est.success);
    EXPECT_EQ(est.inliers.size(), s.matches.size());

    rtvm::Pose error = s.true_motion.inverse() * est.motion;
    EXPECT_LT(error.translation().norm(), 1e-3);                     // under 1 mm
    EXPECT_LT(Eigen::AngleAxisd(error.linear()).angle(), 1e-3);      // under ~0.06 degrees
}

// With 20% wrong matches, RANSAC ignores them and still recovers the exact motion.
TEST(MotionTest, RecoversMotionDespiteOutliers) {
    SyntheticScene s = makeScene(exampleMotion());

    // Corrupt every 5th match (16 of 80) by moving its keypoint in B 40 px to the right.
    for (std::size_t i = 0; i < s.b.keypoints.size(); ++i){
        if (i % 5 == 0){
            s.b.keypoints[i].pt.x += 40.0f;
        }
    }

    rtvm::MotionEstimate est = rtvm::estimateMotion(s.a, s.b, s.matches, s.depth_a, s.camera);
    ASSERT_TRUE(est.success);

    rtvm::Pose error = s.true_motion.inverse() * est.motion;
    EXPECT_LT(error.translation().norm(), 1e-3);                     // under 1 mm
    EXPECT_LT(Eigen::AngleAxisd(error.linear()).angle(), 1e-3);      // under ~0.06 degrees

    EXPECT_EQ(est.inliers.size(), 64u);
}