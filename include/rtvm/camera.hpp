#pragma once

#include <Eigen/Core>

namespace rtvm {

// Pinhole camera model: converts between 3D points (camera frame) and pixels.
struct PinholeCamera {
    double fx;  // focal length in pixels (x)
    double fy;  // focal length in pixels (y)
    double cx;  // principal point (x)
    double cy;  // principal point (y)

    // constructor 
    PinholeCamera(double fx_in, double fy_in, double cx_in, double cy_in);

    // 3D point (X, Y, Z) -> pixel (u, v). Z must be > 0.
    Eigen::Vector2d project(const Eigen::Vector3d& point) const;

    // Pixel (u, v) + depth Z -> 3D point (X, Y, Z).
    Eigen::Vector3d backproject(const Eigen::Vector2d& pixel, double depth) const;
};

// Read fx, fy, cx, cy from a YAML file. Throws if the file or a value is missing.
PinholeCamera loadCameraFromYaml(const std::string& path);

}  // namespace rtvm