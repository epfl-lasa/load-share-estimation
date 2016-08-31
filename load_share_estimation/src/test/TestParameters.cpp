// Author: Felix Duvallet

#include <gtest/gtest.h>
#include <memory>
#include <load_share_estimation/LoadShareParameters.h>
#include <load_share_estimation/LoadShareEstimator.h>

using namespace load_share_estimation;

namespace {

class BasicTest: public ::testing::Test {
  void SetUp() {

  }

 public:
  std::unique_ptr<LoadShareEstimator> worker;
};

TEST_F(BasicTest, TestSomething) {
  ASSERT_EQ(true, true);
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

