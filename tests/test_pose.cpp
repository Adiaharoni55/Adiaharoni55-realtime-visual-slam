#include <gtest/gtest.h>
#include "rtvm/pose.hpp"

// TUM quaternion (0, 0, 0, 1) means "no rotation": the pose is just a shift to (1, 2, 3).
TEST(PoseTest, PoseFromTumWithoutRotation) {
    // 1. Call: position (1, 2, 3), quaternion qx=0 qy=0 qz=0 qw=1
    rtvm::Pose pose = rtvm::poseFromTum(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0);

    // 2. Check: rotation is the identity matrix, position is (1, 2, 3)
    EXPECT_TRUE(pose.linear().isApprox(Eigen::Matrix3d::Identity()));
    EXPECT_NEAR(pose.translation().x(), 1.0, 1e-9);
    EXPECT_NEAR(pose.translation().y(), 2.0, 1e-9);
    EXPECT_NEAR(pose.translation().z(), 3.0, 1e-9);
}

TEST(PoseTest, RelativePoseOfPureShift) {
    // a at origin - no rotation
    rtvm::Pose a = rtvm::poseFromTum(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

    // b moved 1 m along x, no rotation.
    rtvm::Pose b = rtvm::poseFromTum(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

    // Calculate motion
    rtvm::Pose motion = rtvm::relativePose(a, b);
    
    // Check: rotation is the identity matrix and translation is (1, 0, 0)
    EXPECT_TRUE(motion.linear().isApprox(Eigen::Matrix3d::Identity()));
    EXPECT_NEAR(motion.translation().x(), 1.0, 1e-9);
    EXPECT_NEAR(motion.translation().y(), 0.0, 1e-9);
    EXPECT_NEAR(motion.translation().z(), 0.0, 1e-9);

}

TEST(PoseTest, PoseFromTumRotatesCorrectly){
    // Make a pose of 90° around Z in TUM format: position (0, 0, 0), quaternion qx=0, qy=0, qz=0.70710678, qw=0.70710678
    rtvm::Pose pose = rtvm::poseFromTum(0.0, 0.0, 0.0, 0.0, 0.0, 0.70710678, 0.70710678);
    Eigen::Vector3d result = pose.linear() * Eigen::Vector3d(1, 0, 0);
    EXPECT_NEAR(result.x(), 0.0, 1e-9);
    EXPECT_NEAR(result.y(), 1.0, 1e-9);
    EXPECT_NEAR(result.z(), 0.0, 1e-9);
}
