#include <gtest/gtest.h>
#include "frontier_exploration_ros2/observation_progress.hpp"
using namespace frontier_exploration_ros2;

static nav_msgs::msg::OccupancyGrid grid()
{
  nav_msgs::msg::OccupancyGrid m;
  m.info.width=60;m.info.height=30;m.info.resolution=.1;
  m.info.origin.orientation.w=1;
  m.data.assign(1800,-1);
  return m;
}

TEST(ObservationProgress, RepeatedNavigationSuccessWithoutGainCoolsFrontierAndObservation)
{
  ObservationProgress p;auto m=grid();
  auto a=p.capture(OccupancyGrid2d(m),{2,1},{1,1});
  EXPECT_FALSE(p.complete(a,OccupancyGrid2d(m),0).cooling);
  auto b=p.capture(OccupancyGrid2d(m),{2.1,1},{1.1,1});
  EXPECT_TRUE(p.complete(b,OccupancyGrid2d(m),10'000'000'000LL).cooling);
  EXPECT_TRUE(p.blockedReference({2.2,1},11'000'000'000LL));
  EXPECT_TRUE(p.blockedObservation({1.2,1},11'000'000'000LL));
  EXPECT_FALSE(p.blockedReference({4,1},11'000'000'000LL));
  EXPECT_TRUE(p.blockedReference({2,1},131'000'000'000LL));
  p.refresh(OccupancyGrid2d(m),250'000'000'000LL);
  EXPECT_TRUE(p.blockedReference({2,1},250'000'000'000LL));
  EXPECT_FALSE(p.blockedReference({2,1},611'000'000'000LL));
}

TEST(ObservationProgress, NewUnknownCellsRevealedPreventCooldown)
{
  ObservationProgress p;auto m=grid();
  auto a=p.capture(OccupancyGrid2d(m),{2,1},{1,1});
  for(int y=10;y<16;++y)for(int x=10;x<20;++x)m.data[y*60+x]=0;
  auto result=p.complete(a,OccupancyGrid2d(m),0);
  EXPECT_NEAR(result.revealed_area,.60,1e-7);
  EXPECT_EQ(result.no_gain_visits,0U);
}

TEST(ObservationProgress, PreviouslyKnownCellsFlickeringUnknownAreNotNewGain)
{
  ObservationProgress p;auto m=grid();
  for(int i=0;i<20;++i)m.data[10*60+10+i]=0;
  p.capture(OccupancyGrid2d(m),{2,1},{1,1});
  for(int i=0;i<20;++i)m.data[10*60+10+i]=-1;
  auto a=p.capture(OccupancyGrid2d(m),{2,1},{1,1});
  for(int i=0;i<20;++i)m.data[10*60+10+i]=0;
  EXPECT_DOUBLE_EQ(p.complete(a,OccupancyGrid2d(m),0).revealed_area,0.0);
}

TEST(ObservationProgress, InformationOutsideObservedAreaDoesNotMaskLocalLoop)
{
  ObservationProgress p;auto m=grid();
  auto a=p.capture(OccupancyGrid2d(m),{.5,.5},{.5,.5});
  for(int i=40;i<60;++i)m.data[10*60+i]=0;
  EXPECT_DOUBLE_EQ(p.complete(a,OccupancyGrid2d(m),0).revealed_area,0.0);
}

TEST(ObservationProgress, FreshInformationCanReleaseCooldownBeforeExpiry)
{
  ObservationProgress p;auto m=grid();
  for(int i=0;i<2;++i) {
    auto a=p.capture(OccupancyGrid2d(m),{2,1},{1,1});
    p.complete(a,OccupancyGrid2d(m),i*1'000'000'000LL);
  }
  EXPECT_TRUE(p.blockedReference({2,1},2'000'000'000LL));
  for(int y=10;y<16;++y)for(int x=10;x<20;++x)m.data[y*60+x]=0;
  p.refresh(OccupancyGrid2d(m),3'000'000'000LL);
  EXPECT_FALSE(p.blockedReference({2,1},3'000'000'000LL));
}

TEST(ObservationProgress, DifferentFloorOrSessionDoesNotReuseCooldown)
{
  ObservationProgress p;auto m=grid();
  for(int i=0;i<2;++i) {
    auto a=p.capture(OccupancyGrid2d(m),{2,1},{1,1});
    p.complete(a,OccupancyGrid2d(m),i*1'000'000'000LL);
  }
  auto old_floor=p.capture(OccupancyGrid2d(m),{2,1},{1,1});
  m.info.origin.position.z=2.0;
  p.refresh(OccupancyGrid2d(m),2'000'000'000LL);
  EXPECT_FALSE(p.blockedReference({2,1},2'000'000'000LL));
  EXPECT_EQ(p.complete(old_floor,OccupancyGrid2d(m),3'000'000'000LL).no_gain_visits,0U);
  p.clear(); EXPECT_FALSE(p.blockedObservation({1,1},2'000'000'000LL));
}

TEST(ObservationProgress, FirstTraversalOfDifferentKnownRoutePositionsIsNotCooled)
{
  ObservationProgress p;auto m=grid();
  for(auto point:{std::make_pair(1.0,1.0),std::make_pair(1.0,2.0),std::make_pair(1.0,3.0)}) {
    auto snapshot=p.capture(OccupancyGrid2d(m),{2,1},point);
    auto outcome=p.complete(snapshot,OccupancyGrid2d(m),0);
    EXPECT_EQ(outcome.no_gain_visits,1U);EXPECT_FALSE(outcome.cooling);
  }
}

TEST(ObservationProgress, ReturningToAnUnproductivePositionAfterDetourIsCooled)
{
  ObservationProgress p;auto m=grid();
  for(auto point:{std::make_pair(1.0,1.0),std::make_pair(1.0,2.0)}) {
    auto snapshot=p.capture(OccupancyGrid2d(m),{2,1},point);
    EXPECT_FALSE(p.complete(snapshot,OccupancyGrid2d(m),0).cooling);
  }
  auto returned=p.capture(OccupancyGrid2d(m),{2,1},{1.1,1.0});
  EXPECT_TRUE(p.complete(returned,OccupancyGrid2d(m),1'000'000'000LL).cooling);
}

TEST(ObservationProgress, EachSmallUniqueGainCountsAsInformation)
{
  ObservationProgress p;auto m=grid();
  const std::vector<ObservationProgress::Point> points{{1,1},{1.7,1},{1,1.8}};
  for(std::size_t i=0;i<points.size();++i) {
    auto snapshot=p.capture(OccupancyGrid2d(m),{2,2},points[i]);
    for(int x=20;x<30;++x)m.data[(10+i)*60+x]=0;
    const auto outcome=p.complete(snapshot,OccupancyGrid2d(m),i*1'000'000'000LL);
    EXPECT_NEAR(outcome.revealed_area,.10,1e-7);
    EXPECT_FALSE(outcome.cooling);
    EXPECT_EQ(outcome.no_gain_visits,0u);
  }
  EXPECT_FALSE(p.blockedReference({2.9,2},3'000'000'000LL));
}

TEST(ObservationProgress, KnownDetourCanApproachFrontierAfterOutwardProgressStops)
{
  ObservationProgress p;auto m=grid();
  for(auto point:{std::make_pair(1.,0.),std::make_pair(2.,1.),std::make_pair(1.,1.)}) {
    auto snapshot=p.capture(OccupancyGrid2d(m),{0,0},point);
    EXPECT_FALSE(p.complete(snapshot,OccupancyGrid2d(m),0).cooling);
  }
}

TEST(ObservationProgress, DifferentFrontierFragmentsCannotReuseSameUnproductiveViewpoint)
{
  ObservationProgress p;auto m=grid();
  auto first=p.capture(OccupancyGrid2d(m),{2,1},{1,1});
  EXPECT_FALSE(p.complete(first,OccupancyGrid2d(m),0).cooling);
  auto other=p.capture(OccupancyGrid2d(m),{4,1},{1.1,1});
  EXPECT_TRUE(p.complete(other,OccupancyGrid2d(m),1'000'000'000LL).cooling);
  EXPECT_TRUE(p.blockedObservation({1.05,1},2'000'000'000LL));
}

TEST(ObservationProgress, ExportsOnlyCurrentFloorUnexpiredObservationCooldowns) {
  ObservationProgress p;auto m=grid();
  auto a=p.capture(OccupancyGrid2d(m),{2,1},{1,1});
  p.complete(a,OccupancyGrid2d(m),0);
  EXPECT_TRUE(p.blockedObservationCenters(1).empty());
  auto b=p.capture(OccupancyGrid2d(m),{2,1},{1,1});
  p.complete(b,OccupancyGrid2d(m),10'000'000'000LL);
  auto centers=p.blockedObservationCenters(11'000'000'000LL);
  ASSERT_EQ(centers.size(),1u);EXPECT_EQ(centers.front(),std::make_pair(1.,1.));
  EXPECT_TRUE(p.blockedObservationCenters(611'000'000'000LL).empty());
  m.info.origin.position.z=3;
  p.refresh(OccupancyGrid2d(m),12'000'000'000LL);
  EXPECT_TRUE(p.blockedObservationCenters(12'000'000'000LL).empty());
}

TEST(ObservationProgress, EarlierFailedPositionsStayBlockedAfterMovingElsewhere) {
 ObservationProgress p;auto m=grid();
 auto visit=[&](double x,double y,int64_t now) {
   auto s=p.capture(OccupancyGrid2d(m),{2,1},{x,y});return p.complete(s,OccupancyGrid2d(m),now);
 };
 visit(1,1,0);visit(1.1,1,1'000'000'000LL);
 EXPECT_TRUE(p.blockedObservation({1,1},2'000'000'000LL));
 visit(1.2,1,3'000'000'000LL);
 EXPECT_TRUE(p.blockedObservation({1,1},4'000'000'000LL));
 EXPECT_FALSE(p.blockedObservationCenters(4'000'000'000LL).empty());
}
TEST(ObservationProgress, SmallResidualGainCanReleaseCooldown) {
 ObservationProgress p;auto m=grid();m.data.assign(1800,0);
 for(int x=10;x<14;++x)m.data[10*60+x]=-1;
 for(int i=0;i<2;++i) {
   auto s=p.capture(OccupancyGrid2d(m),{1.15,1.05},{1,1});
   p.complete(s,OccupancyGrid2d(m),i*1'000'000'000LL);
 }
 EXPECT_TRUE(p.blockedObservation({1,1},2'000'000'000LL));
 auto s=p.capture(OccupancyGrid2d(m),{1.15,1.05},{1,1});
 for(int x=10;x<14;++x)m.data[10*60+x]=0;
 auto result=p.complete(s,OccupancyGrid2d(m),3'000'000'000LL);
 EXPECT_NEAR(result.revealed_area,.04,1e-6);EXPECT_EQ(result.no_gain_visits,0u);
 EXPECT_FALSE(p.blockedObservation({1,1},4'000'000'000LL));
}
TEST(ObservationProgress, KnownSpaceCircuitIsNotProgressAwayFromFirstPose) {
 ObservationProgress p;auto m=grid();m.data.assign(1800,0);
 auto a=p.capture(OccupancyGrid2d(m),{5,1},{2,1});p.complete(a,OccupancyGrid2d(m),0);
 auto b=p.capture(OccupancyGrid2d(m),{5,1},{1,1});
 EXPECT_TRUE(p.complete(b,OccupancyGrid2d(m),1'000'000'000LL).cooling);
}
TEST(ObservationProgress, RealKnownDetourReducesMappedRemainingDistance) {
 ObservationProgress p;auto m=grid();m.data.assign(1800,0);
 for(int y=0;y<24;++y)m.data[y*60+30]=100;
 auto a=p.capture(OccupancyGrid2d(m),{4,1},{2,1});p.complete(a,OccupancyGrid2d(m),0);
 auto b=p.capture(OccupancyGrid2d(m),{4,1},{2,2});
 EXPECT_FALSE(p.complete(b,OccupancyGrid2d(m),1'000'000'000LL).cooling);
}
