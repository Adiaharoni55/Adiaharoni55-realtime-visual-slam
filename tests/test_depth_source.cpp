#include <gtest/gtest.h>

#include <filesystem>
#include <memory>

#include <opencv2/imgcodecs.hpp>

#include "rtvm/depth_source.hpp"

// A 4x4 depth image with one valid pixel becomes exactly one 3D point.
TEST(DepthSourceTest, TumDepthImageBecomesPoints) {
    // Write a tiny 16-bit depth image: all zeros except pixel (row 2, col 3) = 10000 (2 m).
    std::filesystem::path dir = std::filesystem::temp_directory_path() / "rtvm_depth_test";
    std::filesystem::create_directories(dir / "depth");
    cv::Mat depth(4, 4, CV_16UC1, cv::Scalar(0));
    depth.at<std::uint16_t>(2, 3) = 10000;
    cv::imwrite((dir / "depth" / "d.png").string(), depth);

    rtvm::PinholeCamera cam(500.0, 500.0, 0.0, 0.0);
    std::vector<rtvm::Frame> frames = {{1.0, "rgb/x.png", "depth/d.png", rtvm::Pose::Identity()}};
    rtvm::TumDepthSource source(dir.string(), frames, cam, /*stride=*/1);

    rtvm::PointCloud cloud = source.cloud(0);

    ASSERT_EQ(cloud.points.size(), 1u);
    // backproject(u=3, v=2, z=2): x = 3*2/500, y = 2*2/500, z = 2
    EXPECT_NEAR(cloud.points[0].x(), 0.012, 1e-9);
    EXPECT_NEAR(cloud.points[0].y(), 0.008, 1e-9);
    EXPECT_NEAR(cloud.points[0].z(), 2.0, 1e-9);
}

// A fake source returns exactly the clouds it was given.
TEST(DepthSourceTest, FakeSourceReturnsGivenClouds) {
    // Prepare: one cloud with timestamp 5.0 and two (arbitrary) points
    rtvm::PointCloud clouds;
    clouds.timestamp = 5.0;
    clouds.points.push_back(Eigen::Vector3d(1.0, 2.0, 3.0));   // point 1: x=1, y=2, z=3
    clouds.points.push_back(Eigen::Vector3d(4.0, 5.0, 6.0));   // point 2: x=4, y=5, z=6

    // Call: put it in a fake source ({c} = a vector holding that one cloud)
    rtvm::FakeDepthSource source({clouds});

    // Check: we get back exactly what we gave
    ASSERT_EQ(source.size(), 1u);
    EXPECT_NEAR(source.cloud(0).timestamp, 5.0, 1e-9);
    EXPECT_EQ(source.cloud(0).points.size(), 2u);
    EXPECT_NEAR(source.cloud(0).points[1].z(), 6.0, 1e-9);
    
}

// Code that only knows "a DepthSource" works with any implementation.
TEST(DepthSourceTest, WorksThroughTheInterface) {
    // Prepare: one empty cloud with timestamp 7.0
    rtvm::PointCloud c;
    c.timestamp = 7.0;

    // Call: hold the fake source through the interface type (DepthSource)
    std::unique_ptr<rtvm::DepthSource> source =
        std::make_unique<rtvm::FakeDepthSource>(std::vector<rtvm::PointCloud>{c});

    // Check: calls through the interface reach FakeDepthSource's functions
    ASSERT_EQ(source->size(), 1u);
    EXPECT_NEAR(source->cloud(0).timestamp, 7.0, 1e-9);
}