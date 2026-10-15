#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "rtvm/camera.hpp"
#include "rtvm/features.hpp"
#include "rtvm/pose.hpp"

namespace rtvm {

// Result of estimating how the camera moved from frame A to frame B.
struct MotionEstimate {
    bool success = false;        // false if there were too few usable matches
    Pose motion = Pose::Identity();  // camera B's pose seen from camera A (compare with relativePose)
    std::vector<int> inliers;    // indices into the matches list that RANSAC accepted
    int num_points = 0;          // matches that had valid depth and were given to PnP
};

// Camera matrix K and distortion coefficients in OpenCV's format.
cv::Mat cameraMatrix(const PinholeCamera& camera);
cv::Mat distortionCoeffs(const PinholeCamera& camera);

// Estimate the camera motion from frame A to frame B, using A's depth image
// (16-bit TUM format) to get 3D points, and PnP + RANSAC on the matches.
MotionEstimate estimateMotion(const FrameFeatures& a, const FrameFeatures& b,
                              const std::vector<cv::DMatch>& matches,
                              const cv::Mat& depth_a, const PinholeCamera& camera);

}  // namespace rtvm