#pragma once

#include "rtvm/camera.hpp"
#include "rtvm/features.hpp"
#include "rtvm/pose.hpp"

namespace rtvm {

// Frame-to-frame visual odometry: estimates the camera pose of each new frame
// by matching it against the previous one and chaining the motions.
class VisualOdometry {
public:
    explicit VisualOdometry(PinholeCamera camera);

    // Process the next frame (grayscale image + 16-bit TUM depth image).
    // Returns the camera pose of this frame, relative to the first frame.
    Pose track(const cv::Mat& gray, const cv::Mat& depth);

    // How many frames so far had a failed motion estimate (pose was kept unchanged).
    int failures() const { return failures_; }

private:
    PinholeCamera camera_;
    Pose pose_ = Pose::Identity();   // current camera pose (first frame = origin)
    bool has_previous_ = false;      // false until the first frame has been seen
    FrameFeatures previous_features_;
    cv::Mat previous_depth_;
    int failures_ = 0;
};

}  // namespace rtvm
