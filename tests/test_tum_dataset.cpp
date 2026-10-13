#include <gtest/gtest.h>
#include <stdexcept>
#include <string>
#include "rtvm/tum_dataset.hpp"

// Reading the sample list keeps the 2 data lines and skips the 3 comment lines.
TEST(TumDatasetTest, ReadsFileListSkippingComments) {
    std::string path = std::string(RTVM_TEST_DATA_DIR) + "/rgb_sample.txt";
    std::vector<rtvm::TimedFile> files = rtvm::readTumFileList(path);

    // Stop here if the count is wrong: the checks below read files[0].
    ASSERT_EQ(files.size(), 2u);

    EXPECT_NEAR(files[0].timestamp, 1305031102.175304, 1e-6);
    EXPECT_EQ(files[0].path, "rgb/1305031102.175304.png");
}

// A list file that doesn't exist throws instead of returning an empty list.
TEST(TumDatasetTest, MissingFileListThrows) {
    std::string path = std::string(RTVM_TEST_DATA_DIR) + "/no_such_file.txt";
    EXPECT_THROW(rtvm::readTumFileList(path), std::runtime_error);
}

// Reading the sample ground truth keeps the 2 data lines and parses each into a pose
TEST(TumDatasetTest, ReadsGroundTruthSkippingComments) {
    std::string path = std::string(RTVM_TEST_DATA_DIR) + "/groundtruth_sample.txt";
    std::vector<rtvm::TimedPose> poses = rtvm::readTumGroundTruth(path);

    // Stop here if the count is wrong: the checks below read poses[0].
    ASSERT_EQ(poses.size(), 2u);
    
    EXPECT_NEAR(poses[0].timestamp, 1305031098.6659, 1e-6);
    EXPECT_NEAR(poses[0].pose.translation().x(), 1.3563, 1e-9);
    EXPECT_NEAR(poses[0].pose.translation().y(), 0.6305, 1e-9);
    EXPECT_NEAR(poses[0].pose.translation().z(), 1.6380, 1e-9);
    
}

// A ground truth file that doesn't exist throws instead of returning an empty list.
TEST(TumDatasetTest, MissingGroundTruthThrows) {
    std::string path = std::string(RTVM_TEST_DATA_DIR) + "/no_such_file.txt";
    EXPECT_THROW(rtvm::readTumGroundTruth(path), std::runtime_error);
}
