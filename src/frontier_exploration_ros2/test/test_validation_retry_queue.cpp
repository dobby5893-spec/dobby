#include <gtest/gtest.h>
#include "frontier_exploration_ros2/validation_retry_queue.hpp"

using frontier_exploration_ros2::ValidationRetryQueue;
static auto at(int seconds)
{return ValidationRetryQueue::Time{} + std::chrono::seconds(seconds);}

TEST(ValidationRetryQueue, SlowFirstCandidateDoesNotStarveAlternatives)
{
  ValidationRetryQueue q;
  q.enqueue("near_wall", at(0));
  ASSERT_EQ(q.next(at(0))->key, "near_wall");
  q.enqueue("near_wall", at(1));
  q.enqueue("open_corridor", at(1));
  q.enqueue("other_door", at(1));
  EXPECT_FALSE(q.next(at(1)));
  EXPECT_DOUBLE_EQ(q.finish("near_wall", true, at(5)), 10.0);
  q.enqueue("near_wall", at(6));
  auto job = q.next(at(6));
  ASSERT_TRUE(job); EXPECT_EQ(job->key, "open_corridor");
  q.finish(job->key, false, at(7));
  ASSERT_EQ(q.next(at(7))->key, "other_door");
}

TEST(ValidationRetryQueue, RepeatedTimeoutsBackOffAndBudgetIsCapped)
{
  ValidationRetryQueue q(5.0, 10.0);
  q.enqueue("a", at(0));
  EXPECT_DOUBLE_EQ(q.next(at(0))->budget, 5.0);
  q.finish("a", true, at(5)); q.enqueue("a", at(6));
  EXPECT_FALSE(q.next(at(14)));
  auto second = q.next(at(15)); ASSERT_TRUE(second);
  EXPECT_DOUBLE_EQ(second->budget, 7.5); EXPECT_EQ(second->attempt, 2U);
  q.finish("a", true, at(23)); q.enqueue("a", at(52));
  EXPECT_FALSE(q.next(at(52)));
  auto third = q.next(at(53)); ASSERT_TRUE(third);
  EXPECT_DOUBLE_EQ(third->budget, 10.0); EXPECT_EQ(third->attempt, 3U);
  EXPECT_DOUBLE_EQ(q.finish("a", true, at(63)), 120.0);
  q.enqueue("a", at(182)); EXPECT_FALSE(q.next(at(182)));
  EXPECT_TRUE(q.next(at(183)));  // temporary deferment, not a permanent blacklist
}

TEST(ValidationRetryQueue, CoolingCandidatesAreNotReportedAsRunningWork)
{
  ValidationRetryQueue q; q.enqueue("a", at(0)); q.next(at(0));
  q.finish("a", true, at(5)); q.enqueue("a", at(6));
  EXPECT_FALSE(q.active()); EXPECT_FALSE(q.hasReady(at(6)));
  q.enqueue("b", at(6)); EXPECT_TRUE(q.hasReady(at(6)));
}

TEST(ValidationRetryQueue, DuplicateRequestsDoNotDisplaceQueuedAlternative)
{
  ValidationRetryQueue q;
  q.enqueue("a", at(0)); q.enqueue("b", at(0));
  for (int i = 0; i < 20; ++i) {q.enqueue("a", at(0));}
  auto a = q.next(at(0)); ASSERT_TRUE(a); q.finish(a->key, true, at(5));
  ASSERT_EQ(q.next(at(5))->key, "b");
}

TEST(ValidationRetryQueue, UnselectedStaleRequestIsNotLaunched)
{
  ValidationRetryQueue q; q.enqueue("old_goal", at(0));
  EXPECT_FALSE(q.next(at(31)));
  q.enqueue("new_goal", at(31)); ASSERT_EQ(q.next(at(31))->key, "new_goal");
}

TEST(ValidationRetryQueue, RestartClearsPreviousSessionWorkAndBackoff)
{
  ValidationRetryQueue q; q.enqueue("a", at(0)); q.next(at(0));
  q.finish("a", true, at(5)); q.enqueue("a", at(6)); q.clear();
  q.enqueue("a", at(7)); auto job = q.next(at(7)); ASSERT_TRUE(job);
  EXPECT_EQ(job->attempt, 1U); EXPECT_DOUBLE_EQ(job->budget, 5.0);
}

TEST(ValidationRetryQueue, CompletedNoRouteResultsBackOffWithoutBlockingOtherGoals) {
 ValidationRetryQueue q;q.enqueue("blocked",at(0));ASSERT_TRUE(q.next(at(0)));
 EXPECT_DOUBLE_EQ(q.finish("blocked",false,at(0),false),5);
 q.enqueue("blocked",at(1));q.enqueue("other",at(1));
 auto job=q.next(at(1));ASSERT_TRUE(job);EXPECT_EQ(job->key,"other");
 q.finish("other",false,at(1),true);
 auto retry=q.next(at(5));ASSERT_TRUE(retry);EXPECT_EQ(retry->attempt,2u);
 EXPECT_DOUBLE_EQ(q.finish("blocked",false,at(5),false),15);
 q.enqueue("blocked",at(19));EXPECT_FALSE(q.next(at(19)));ASSERT_TRUE(q.next(at(20)));
 EXPECT_DOUBLE_EQ(q.finish("blocked",false,at(20),false),60);
 q.enqueue("blocked",at(79));EXPECT_FALSE(q.next(at(79)));ASSERT_TRUE(q.next(at(80)));
 EXPECT_DOUBLE_EQ(q.finish("blocked",false,at(80),false),120);
 q.enqueue("blocked",at(199));EXPECT_FALSE(q.next(at(199)));ASSERT_TRUE(q.next(at(200)));
 q.finish("blocked",false,at(200),true);q.enqueue("blocked",at(203));
 auto fresh=q.next(at(203));ASSERT_TRUE(fresh);EXPECT_EQ(fresh->attempt,1u);
}
