#include <gtest/gtest.h>
#include "frontier_exploration_ros2/localization_freshness.hpp"
using frontier_exploration_ros2::LocalizationFreshness;
using State=LocalizationFreshness::State;
TEST(LocalizationFreshness, StartupAllowsDiscoveryButDoesNotWaitForever) {
  LocalizationFreshness f;f.reset(0);
  EXPECT_EQ(f.observe(29'000'000'000LL,std::nullopt),State::Waiting);
  EXPECT_EQ(f.observe(30'000'000'000LL,std::nullopt),State::Expired);
}
TEST(LocalizationFreshness, StopsAfterPreviouslyHealthyLocalizationIsLost) {
  LocalizationFreshness f;f.reset(0);
  EXPECT_EQ(f.observe(1'000'000'000LL,1'000'000'000LL),State::Healthy);
  EXPECT_EQ(f.observe(4'000'000'000LL,1'000'000'000LL),State::Waiting);
  EXPECT_EQ(f.observe(6'000'000'000LL,1'000'000'000LL),State::Expired);
}
TEST(LocalizationFreshness, PauseDoesNotAdvanceTimeout) {
  LocalizationFreshness f;f.reset(0);f.observe(1'000'000'000LL,1'000'000'000LL);
  for(int i=0;i<1000;++i)EXPECT_EQ(f.observe(1'000'000'000LL,1'000'000'000LL),State::Healthy);
}
TEST(LocalizationFreshness, ClockRewindResetsGraceAndFreshDataCanRecover) {
  LocalizationFreshness f;f.reset(0);f.observe(100'000'000'000LL,100'000'000'000LL);
  EXPECT_EQ(f.observe(0,std::nullopt),State::Waiting);
  EXPECT_EQ(f.observe(1'000'000'000LL,1'000'000'000LL),State::Healthy);
}
TEST(LocalizationFreshness, RejectsOldAndFarFutureTransforms) {
  EXPECT_FALSE(LocalizationFreshness::accepts(10'000'000'000LL,1'000'000'000LL));
  EXPECT_FALSE(LocalizationFreshness::accepts(10'000'000'000LL,20'000'000'000LL));
  EXPECT_TRUE(LocalizationFreshness::accepts(10'000'000'000LL,9'000'000'000LL));
}
