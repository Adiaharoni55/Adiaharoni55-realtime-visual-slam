#include "rtvm/visual_odometry.hpp"

#include <utility>
#include <vector>

#include "rtvm/motion.hpp"

namespace rtvm {
VisualOdometry::VisualOdometry(PinholeCamera camera) 
    : camera_(std::move(camera)) {}

Pose VisualOdometry::track(const cv::Mat& gray, const cv::Mat& depth){
    FrameFeatures current = detectOrb(gray);

    // From the second frame on: estimate the motion since the previous frame and chain it.
    if (has_previous_){
        std::vector<cv::DMatch> matches = matchFeatures(previous_features_, current);
        MotionEstimate est = estimateMotion(previous_features_, current, matches,
                                            previous_depth_, camera_);
        if (est.success){
        pose_ = pose_ * est.motion;
        } else {
            ++failures_;
        }
    }
    // Remember this frame for the next call (also on the very first frame).
    previous_features_ = std::move(current);
    previous_depth_ = depth.clone();
    has_previous_ = true;
    return pose_;
}

} // namespace rtvm
