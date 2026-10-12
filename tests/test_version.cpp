#include <gtest/gtest.h>
#include "rtvm/version.hpp"

TEST(VersionTest, ReturnsProjectVersion) {
    EXPECT_EQ(rtvm::version(), "0.0.1");
}