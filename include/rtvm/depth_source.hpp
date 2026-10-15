#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "rtvm/camera.hpp"
#include "rtvm/pose.hpp"
#include "rtvm/tum_dataset.hpp"

namespace rtvm {

// 3D points from one depth measurement (a depth image or a LiDAR scan).
struct PointCloud {
    double timestamp = 0.0;                  // seconds
    std::vector<Eigen::Vector3d> points;     // in the sensor's own frame (meters)
    Pose sensor_to_camera = Pose::Identity(); // where the sensor sits relative to the camera
};

// Anything that produces point clouds. The pipeline only uses this interface,
// so it never needs to know which sensor it is reading.
class DepthSource {
public:
    virtual ~DepthSource() = default;

    // Number of clouds available.
    virtual std::size_t size() const = 0;

    // The i-th cloud (0 <= i < size()).
    virtual PointCloud cloud(std::size_t i) const = 0;
};

// Depth source for TUM RGB-D: turns each depth image into a point cloud.
class TumDepthSource : public DepthSource {
public:
    // sequence_dir: folder with depth/ ; frames: from associateTum ;
    // stride: use every stride-th pixel (1 = all pixels, 4 = 16x fewer points).
    TumDepthSource(std::string sequence_dir, std::vector<Frame> frames,
                   PinholeCamera camera, int stride = 4);

    std::size_t size() const override;
    PointCloud cloud(std::size_t i) const override;

private:
    std::string sequence_dir_;
    std::vector<Frame> frames_;
    PinholeCamera camera_;
    int stride_;
};

// Depth source for tests: returns the clouds it was given, so code can run without real files.
class FakeDepthSource : public DepthSource {
public:
    explicit FakeDepthSource(std::vector<PointCloud> clouds);
    
    std::size_t size() const override;
    PointCloud cloud(std::size_t i) const override;

private:
    std::vector<PointCloud> clouds_;
};

}  // namespace rtvm