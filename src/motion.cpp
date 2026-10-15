#include "rtvm/motion.hpp"

#include <cmath>
#include <cstdint>

#include <opencv2/calib3d.hpp>

namespace rtvm {

cv::Mat cameraMatrix(const PinholeCamera& camera) {
    // cv::Matx33d is a fixed 3x3 matrix; cv::Mat(...) copies it into a regular OpenCV matrix.
    cv::Matx33d K(camera.fx, 0.0,       camera.cx,
                  0.0,       camera.fy, camera.cy,
                  0.0,       0.0,       1.0);
    return cv::Mat(K);
}

cv::Mat distortionCoeffs(const PinholeCamera& camera) {
    // A 1x5 matrix of doubles, filled one value at a time.
    cv::Mat D(1, 5, CV_64F);
    for (int i = 0; i < 5; ++i) {
        D.at<double>(0, i) = camera.distortion[i];
    }
    return D;
}

// Estimate camera motion A -> B: 3D points from A's depth, PnP + RANSAC against B's pixels.
MotionEstimate estimateMotion(const FrameFeatures& a, const FrameFeatures& b,
                              const std::vector<cv::DMatch>& matches,
                              const cv::Mat& depth_a, const PinholeCamera& camera) {
    MotionEstimate result;

    // 1. Keep only matches whose keypoint in A has a valid depth.
    std::vector<cv::Point2f> pixels_a;   // keypoint in A (raw pixel)
    std::vector<float> depths_a;         // its depth in meters
    std::vector<cv::Point2f> pixels_b;   // the matching keypoint in B
    std::vector<int> used_matches;       // index into matches, for each kept point

    for (std::size_t k = 0; k < matches.size(); ++k) {
        const cv::DMatch& m = matches[k];
        cv::Point2f pa = a.keypoints[m.queryIdx].pt;
        cv::Point2f pb = b.keypoints[m.trainIdx].pt;
        int u = std::lround(pa.x);
        int v = std::lround(pa.y);

        // Rounding can land one pixel past the edge; skip those keypoints.
        if (u < 0 || v < 0 || u >= depth_a.cols || v >= depth_a.rows) {
            continue;
        }

        std::uint16_t raw = depth_a.at<std::uint16_t>(v, u);
        if (raw == 0) {
            continue;   // no depth measured at this pixel
        }

        pixels_a.push_back(pa);
        pixels_b.push_back(pb);
        depths_a.push_back(raw / 5000.0f);   // TUM: 5000 = 1 meter
        used_matches.push_back(static_cast<int>(k));
    }

    // PnP needs enough points to be reliable.
    if (pixels_a.size() < 6) {
        return result;   // success stays false
    }

    // 2. Undistort A's keypoints and back-project them to 3D (in A's camera frame).
    cv::Mat K = cameraMatrix(camera);
    cv::Mat D = distortionCoeffs(camera);
    std::vector<cv::Point2f> undistorted_a;
    cv::undistortPoints(pixels_a, undistorted_a, K, D, cv::noArray(), K);

    std::vector<cv::Point3f> object_points;
    for (std::size_t i = 0; i < undistorted_a.size(); ++i) {
        Eigen::Vector3d X = camera.backproject(
            Eigen::Vector2d(undistorted_a[i].x, undistorted_a[i].y), depths_a[i]);
        object_points.push_back(cv::Point3f(X.x(), X.y(), X.z()));
    }

    // 3. PnP + RANSAC: the transform that maps the 3D points onto B's pixels.
    //    100 iterations, inlier if within 2 px, 99% confidence. D handles B's distortion.
    cv::Mat rvec, tvec;
    std::vector<int> inliers;
    bool ok = cv::solvePnPRansac(object_points, pixels_b, K, D, rvec, tvec,
                                 false, 100, 2.0f, 0.99, inliers);
    if (!ok) {
        return result;   // success stays false
    }

    // 4. Convert OpenCV's (rvec, tvec) into a Pose. This maps points from A's frame
    //    into B's frame; its inverse is the camera's motion (B seen from A).
    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);   // rotation vector -> 3x3 rotation matrix
    Pose a_to_b = Pose::Identity();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            a_to_b.linear()(r, c) = R_cv.at<double>(r, c);
        }
        a_to_b.translation()(r) = tvec.at<double>(r);
    }

    // 5. Fill the result; translate inlier indices back into indices of `matches`.
    for (int i : inliers) {
        result.inliers.push_back(used_matches[i]);
    }
    result.success = true;
    result.motion = a_to_b.inverse();
    result.num_points = static_cast<int>(object_points.size());
    return result;
}

}  // namespace rtvm
