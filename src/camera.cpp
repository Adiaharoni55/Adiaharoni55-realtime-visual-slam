#include "rtvm/camera.hpp"
#include <yaml-cpp/yaml.h>
#include <stdexcept>
#include <vector>

namespace rtvm {

// constructor 
PinholeCamera::PinholeCamera(double fx_in, double fy_in, double cx_in, double cy_in,
                             std::array<double, 5> distortion_in)
    : fx(fx_in), fy(fy_in), cx(cx_in), cy(cy_in), distortion(distortion_in) {
    if (fx <= 0 || fy <= 0) {
        throw std::invalid_argument("PinholeCamera: fx and fy must be > 0");
    }
}

// 3D point (X, Y, Z) -> pixel (u, v). Z must be > 0.
Eigen::Vector2d PinholeCamera::project(const Eigen::Vector3d& point) const{
    if (point.z() <= 0){
        throw std::invalid_argument("project: point must be in front of the camera (Z > 0)");
    }
    double u = fx * point.x() / point.z() + cx;
    double v = fy * point.y() / point.z() + cy;
    return Eigen::Vector2d(u, v);
}

// Pixel (u, v) + depth Z -> 3D point (X, Y, Z).
Eigen::Vector3d PinholeCamera::backproject(const Eigen::Vector2d& pixel, double depth) const{
    if (depth <= 0){
        throw std::invalid_argument("backproject: depth must be > 0");
    }
    double x = (pixel.x() - cx) * depth / fx;
    double y = (pixel.y() - cy) * depth / fy;
    return Eigen::Vector3d(x, y, depth);
}

PinholeCamera loadCameraFromYaml(const std::string& path) {
    YAML::Node config = YAML::LoadFile(path);

    // Distortion is optional: missing means a perfect lens (all zeros).
    std::array<double, 5> distortion{};
    if (config["distortion"]) {
        std::vector<double> values = config["distortion"].as<std::vector<double>>();
        if (values.size() != 5) {
            throw std::invalid_argument("loadCameraFromYaml: distortion needs 5 values");
        }
        for (std::size_t i = 0; i < 5; ++i) {
            distortion[i] = values[i];
        }
    }

    return PinholeCamera(config["fx"].as<double>(), config["fy"].as<double>(),
                         config["cx"].as<double>(), config["cy"].as<double>(),
                         distortion);
}

} // namespace rtvm
