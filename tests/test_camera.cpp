#include <gtest/gtest.h>
#include <stdexcept>
#include <string>
#include "rtvm/camera.hpp"

// A point 2 m ahead, a bit right and down, lands at pixel (370, 265).
TEST(PinholeCameraTest, ProjectsKnownPoint) {
    rtvm::PinholeCamera cam(500.0, 500.0, 320.0, 240.0);
    Eigen::Vector2d pixel = cam.project(Eigen::Vector3d(0.2, 0.1, 2.0));
    EXPECT_NEAR(pixel.x(), 370.0, 1e-9);
    EXPECT_NEAR(pixel.y(), 265.0, 1e-9);
}

// A point straight ahead of the camera lands on the principal point (cx, cy).
TEST(PinholeCameraTest, PointOnAxisProjectsToPrincipalPoint) {
    rtvm::PinholeCamera cam(500.0, 500.0, 320.0, 240.0);
    Eigen::Vector2d pixel = cam.project(Eigen::Vector3d(0.0, 0.0, 1.0));
    EXPECT_NEAR(pixel.x(), 320.0, 1e-9);
    EXPECT_NEAR(pixel.y(), 240.0, 1e-9);
}

// Projecting and then back-projecting with the same depth returns the original point.
TEST(PinholeCameraTest, BackprojectInvertsProject) {
    rtvm::PinholeCamera cam(500.0, 500.0, 320.0, 240.0);
    Eigen::Vector3d original(0.2, 0.1, 2.0);
    Eigen::Vector2d pixel = cam.project(original);
    Eigen::Vector3d back = cam.backproject(pixel, 2.0);
    EXPECT_NEAR(back.x(), original.x(), 1e-9);
    EXPECT_NEAR(back.y(), original.y(), 1e-9);
    EXPECT_NEAR(back.z(), original.z(), 1e-9);
}

// Invalid camera parameters or inputs throw instead of returning garbage.
TEST(PinholeCameraTest, RejectsInvalidInput) {
    EXPECT_THROW(rtvm::PinholeCamera(0.0, 500.0, 320.0, 240.0), std::invalid_argument);

    rtvm::PinholeCamera cam(500.0, 500.0, 320.0, 240.0);
    EXPECT_THROW(cam.project(Eigen::Vector3d(0.2, 0.1, -2.0)), std::invalid_argument);
    EXPECT_THROW(cam.backproject(Eigen::Vector2d(370.0, 265.0), -2.0), std::invalid_argument);
}

// Loading the TUM fr1 config gives exactly the values in the file.
TEST(PinholeCameraTest, LoadsFromYaml) {
    std::string path = std::string(RTVM_CONFIG_DIR) + "/tum_fr1.yaml";

    rtvm::PinholeCamera cam = rtvm::loadCameraFromYaml(path);

    EXPECT_NEAR(cam.fx, 517.3, 1e-9);
    EXPECT_NEAR(cam.fy, 516.5, 1e-9);
    EXPECT_NEAR(cam.cx, 318.6, 1e-9);
    EXPECT_NEAR(cam.cy, 255.3, 1e-9);
    EXPECT_NEAR(cam.distortion[0], 0.2624, 1e-9);
    EXPECT_NEAR(cam.distortion[4], 1.1633, 1e-9);
}

// Loading a file that doesn't exist throws instead of returning a broken camera.

TEST(PinholeCameraTest, LoadFromMissingFileThrows) {
    std::string path = std::string(RTVM_CONFIG_DIR) + "/no_such_file.yaml";
    EXPECT_ANY_THROW(rtvm::loadCameraFromYaml(path));
}