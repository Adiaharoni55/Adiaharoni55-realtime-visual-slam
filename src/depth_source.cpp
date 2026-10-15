#include "rtvm/depth_source.hpp"

#include <cstdint>
#include <stdexcept>
#include <utility>

#include <opencv2/imgcodecs.hpp>

namespace rtvm {

TumDepthSource::TumDepthSource(std::string sequence_dir, std::vector<Frame> frames,
                               PinholeCamera camera, int stride)
    : sequence_dir_(std::move(sequence_dir)),
      frames_(std::move(frames)),
      camera_(camera),
      stride_(stride) {
    if (stride_ < 1) {
        throw std::invalid_argument("TumDepthSource: stride must be >= 1");
    }
}

std::size_t TumDepthSource::size() const {
    return frames_.size();
}

PointCloud TumDepthSource::cloud(std::size_t i) const {
    const Frame& frame = frames_.at(i);   // .at() throws if i is out of range

    // TUM depth images are 16-bit PNGs; IMREAD_UNCHANGED keeps all 16 bits.
    cv::Mat depth = cv::imread(sequence_dir_ + "/" + frame.depth_path, cv::IMREAD_UNCHANGED);
    if (depth.empty() || depth.type() != CV_16UC1) {
        throw std::runtime_error("TumDepthSource: cannot read 16-bit depth image " + frame.depth_path);
    }

    PointCloud result;
    result.timestamp = frame.timestamp;

    // TUM stores depth as an integer: 5000 = 1 meter, 0 = no measurement.
    constexpr double kDepthScale = 5000.0;
    for (int v = 0; v < depth.rows; v += stride_) {
        for (int u = 0; u < depth.cols; u += stride_) {
            std::uint16_t raw = depth.at<std::uint16_t>(v, u);
            if (raw == 0) {
                continue;   // missing measurement
            }
            double z = raw / kDepthScale;
            result.points.push_back(camera_.backproject(Eigen::Vector2d(u, v), z));
        }
    }
    return result;
}

FakeDepthSource::FakeDepthSource(std::vector<PointCloud> clouds) : clouds_(std::move(clouds)) {}


std::size_t FakeDepthSource::size() const {
    return clouds_.size();
}

PointCloud FakeDepthSource::cloud(std::size_t i) const {
    return clouds_.at(i);
}

}  // namespace rtvm