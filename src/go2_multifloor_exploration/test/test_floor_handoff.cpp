#include <gtest/gtest.h>
#include <vector>
#include "go2_multifloor_exploration/floor_handoff.hpp"
using namespace go2_multifloor_exploration;
TEST(FloorHandoff, FreshMatchingFloorCanRequestStairEvaluation) {
 auto report=parseFloorExhaustion("v1;floor_z=-.21;stamp=100;reason=closed_regions_excluded;remaining_unknown_area=.42;actionable_unknown_area=0;excluded_unknown_area=.42;attempts=4");
 ASSERT_TRUE(report);EXPECT_TRUE(matchesActiveFloor(*report,-.20,102));
 EXPECT_NEAR(report->remaining_unknown_area,.42,1e-9);
}
TEST(FloorHandoff, RetainedOldFloorOrOldClockCannotTriggerTransition) {
 auto report=parseFloorExhaustion("v1;floor_z=-.21;stamp=100;reason=closed_regions_excluded;remaining_unknown_area=.42;actionable_unknown_area=0;excluded_unknown_area=.42;attempts=4");
 ASSERT_TRUE(report);
 EXPECT_FALSE(matchesActiveFloor(*report,2.2,102));
 EXPECT_FALSE(matchesActiveFloor(*report,-.2,106));
 EXPECT_FALSE(matchesActiveFloor(*report,-.2,0));
}
TEST(FloorHandoff, RejectsMalformedNegativeAndDuplicateReports) {
 for(auto text:{"v1;floor_z=nan;stamp=100;reason=closed_regions_excluded;remaining_unknown_area=.42;actionable_unknown_area=0;excluded_unknown_area=.42;attempts=4",
 "v1;floor_z=-.2;stamp=100;reason=closed_regions_excluded;remaining_unknown_area=-.1;attempts=4",
 "v1;floor_z=-.2;stamp=100;reason=done;remaining_unknown_area=.42;actionable_unknown_area=0;excluded_unknown_area=.42;attempts=4",
 "v1;floor_z=-.2;stamp=100;stamp=101;reason=closed_regions_excluded;remaining_unknown_area=.42;actionable_unknown_area=0;excluded_unknown_area=.42;attempts=4"})
 EXPECT_FALSE(parseFloorExhaustion(text));
}

TEST(FloorHandoff, NoGainTimeoutAndOpenResidualCannotTriggerStairs) {
 EXPECT_FALSE(parseFloorExhaustion("v1;floor_z=0;stamp=100;reason=no_gain_cleanup_budget_exhausted;remaining_unknown_area=31.21;attempts=10"));
 EXPECT_FALSE(parseFloorExhaustion("v1;floor_z=0;stamp=100;reason=closed_regions_excluded;remaining_unknown_area=31.21;actionable_unknown_area=30;excluded_unknown_area=1.21;attempts=10"));
 EXPECT_FALSE(parseFloorExhaustion("v1;floor_z=0;stamp=100;reason=closed_regions_excluded;remaining_unknown_area=31.21;actionable_unknown_area=0;excluded_unknown_area=1.21;attempts=10"));
 EXPECT_TRUE(parseFloorExhaustion("v1;floor_z=0;stamp=100;reason=closed_regions_excluded;remaining_unknown_area=4;actionable_unknown_area=0;excluded_unknown_area=4;attempts=0"));
}

static std::string residualReport() {
 return "v1;floor_z=0;stamp=100;reason=bounded_residuals_deferred;remaining_unknown_area=10.59;actionable_unknown_area=9.38;excluded_unknown_area=1.21;deferred_unknown_area=9.38;evidence_rounds=6;raw_empty_rounds=6;failed_groups=0;failed_options=0;observation_positions=0;expanded_stage=3;healthy_no_gain_s=180;attempts=50";
}
TEST(FloorHandoff, RejectsLegacyGlobalResidualDeferral) {
 EXPECT_FALSE(parseFloorExhaustion(residualReport()));
}
TEST(FloorHandoff, ResidualReasonRequiresCompleteConsistentEvidence) {
 for(auto change:std::vector<std::pair<std::string,std::string>>{
   {"evidence_rounds=6","evidence_rounds=1"},{"expanded_stage=3","expanded_stage=2"},
   {"healthy_no_gain_s=180","healthy_no_gain_s=179"},{"raw_empty_rounds=6","raw_empty_rounds=0"},
   {"deferred_unknown_area=9.38","deferred_unknown_area=8.0"},
   {"remaining_unknown_area=10.59","remaining_unknown_area=20.0"},
   {"evidence_rounds=6","evidence_rounds=6.5"}}) {
  auto text=residualReport();text.replace(text.find(change.first),change.first.size(),change.second);
  EXPECT_FALSE(parseFloorExhaustion(text))<<text;
 }
}

TEST(FloorHandoff, ScopedSingletonFailuresStillCannotProveFloorCompletion) {
 auto text=residualReport();
 for(auto change:std::vector<std::pair<std::string,std::string>>{
   {"raw_empty_rounds=6","raw_empty_rounds=0"},{"failed_groups=0","failed_groups=3"},
   {"failed_options=0","failed_options=3"}})
   text.replace(text.find(change.first),change.first.size(),change.second);
 EXPECT_FALSE(parseFloorExhaustion(text));
 auto scoped=text+";validation_scope=all_candidates";
 EXPECT_FALSE(parseFloorExhaustion(scoped));
 auto partial=scoped;partial.replace(partial.find("failed_options=3"),16,"failed_options=2");
 EXPECT_FALSE(parseFloorExhaustion(partial));
}

TEST(FloorHandoff, ClosedReasonCannotSmuggleUnresolvedDeferredArea) {
 EXPECT_FALSE(parseFloorExhaustion("v1;floor_z=0;stamp=100;reason=closed_regions_excluded;remaining_unknown_area=4;actionable_unknown_area=0;excluded_unknown_area=4;deferred_unknown_area=1;attempts=0"));
 EXPECT_TRUE(parseFloorExhaustion("v1;floor_z=0;stamp=100;reason=closed_regions_excluded;remaining_unknown_area=4;actionable_unknown_area=0;excluded_unknown_area=4;deferred_unknown_area=0;attempts=0"));
}

static std::string regionReport() {
 return "v2;floor_z=0;stamp=100;reason=region_observability_audit;remaining_unknown_area=3;actionable_unknown_area=0;excluded_unknown_area=1;deferred_unknown_area=2;unresolved_unknown_area=0;scope=per_region_current_floor;region_count=2;stable_samples=3;stable_seconds=10;safe_start=1;complete_views=1;attempts=0;regions=11:1:occluded,22:1:no_known_access";
}
TEST(FloorHandoff, IndividualAuditedRegionsCanRequestStairsWithoutClaimingFullCoverage) {
 auto report=parseFloorExhaustion(regionReport());ASSERT_TRUE(report);
 EXPECT_DOUBLE_EQ(report->deferred_unknown_area,2);
 EXPECT_TRUE(matchesActiveFloor(*report,0,101));
}
TEST(FloorHandoff, MissingRegionUncertainRegionOrUnsafeStartBlocksAuditHandoff) {
 for(auto change:std::vector<std::pair<std::string,std::string>>{
  {"region_count=2","region_count=3"},{"stable_samples=3","stable_samples=2"},
  {"stable_seconds=10","stable_seconds=9"},{"safe_start=1","safe_start=0"},
  {"complete_views=1","complete_views=0"},{"unresolved_unknown_area=0","unresolved_unknown_area=1"},
  {"22:1:no_known_access","22:1:undetermined"},{"22:1:no_known_access","22:1:planner_failed"},
  {"22:1:no_known_access","11:1:occluded"},{"22:1:no_known_access","22:2:occluded"},
  {"actionable_unknown_area=0","actionable_unknown_area=1"}}) {
   auto text=regionReport();text.replace(text.find(change.first),change.first.size(),change.second);
   EXPECT_FALSE(parseFloorExhaustion(text))<<text;
 }
}

TEST(FloorHandoff, AcceptsSeparateStairTaskOnlyWithCompleteRouteAudit) {
 const std::string text="v3;floor_z=0;stamp=20;reason=region_observability_audit;remaining_unknown_area=1.5;actionable_unknown_area=0;excluded_unknown_area=0;deferred_unknown_area=.5;transition_unknown_area=1;unresolved_unknown_area=0;scope=per_region_current_floor;region_count=2;stable_samples=3;stable_seconds=10;safe_start=1;complete_views=1;complete_routes=1;attempts=0;regions=1:1:transition,2:.5:no_route";
 auto report=go2_multifloor_exploration::parseFloorExhaustion(text);ASSERT_TRUE(report);
 EXPECT_DOUBLE_EQ(report->transition_unknown_area,1);
 auto broken=text;broken.replace(broken.find("complete_routes=1"),17,"complete_routes=0");
 EXPECT_FALSE(go2_multifloor_exploration::parseFloorExhaustion(broken));
 auto wrong=text;wrong.replace(wrong.find("transition_unknown_area=1"),25,"transition_unknown_area=2");
 EXPECT_FALSE(go2_multifloor_exploration::parseFloorExhaustion(wrong));
}

TEST(FloorHandoff, V4TaskExhaustionKeepsVerificationUncertaintyExplicit) {
 const std::string report="v4;floor_z=0;stamp=20;reason=region_observability_audit;"
 "remaining_unknown_area=.3;actionable_unknown_area=0;excluded_unknown_area=0;"
 "deferred_unknown_area=.3;transition_unknown_area=0;unresolved_unknown_area=0;"
 "scope=per_region_current_floor;region_count=2;stable_samples=3;stable_seconds=10;"
 "safe_start=1;complete_views=1;complete_routes=0;runtime_evidence=1;"
 "runtime_min_positions=3;runtime_min_healthy_s=60;attempts=0;"
 "regions=11:.15:observed_no_gain,22:.15:verification_deferred";
 EXPECT_TRUE(parseFloorExhaustion(report));
 auto weak=report;auto at=weak.find("runtime_min_positions=3");
 weak.replace(at,std::string("runtime_min_positions=3").size(),"runtime_min_positions=1");
 EXPECT_FALSE(parseFloorExhaustion(weak));
 auto stale=report;at=stale.find("runtime_min_healthy_s=60");
 stale.replace(at,std::string("runtime_min_healthy_s=60").size(),"runtime_min_healthy_s=5");
 EXPECT_FALSE(parseFloorExhaustion(stale));
 auto legacy=report;legacy.replace(0,2,"v3");EXPECT_FALSE(parseFloorExhaustion(legacy));
 auto lying=report;at=lying.find("verification_deferred");
 lying.replace(at,std::string("verification_deferred").size(),"no_route");
 EXPECT_FALSE(parseFloorExhaustion(lying));
}
