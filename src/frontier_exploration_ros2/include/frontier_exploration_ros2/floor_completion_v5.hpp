#pragma once

#include "frontier_exploration_ros2/floor_completion.hpp"
#include "frontier_exploration_ros2/floor_session.hpp"
#include "frontier_exploration_ros2/map_consistency.hpp"
#include "frontier_exploration_ros2/msg/floor_completion.hpp"
#include <unordered_set>

namespace frontier_exploration_ros2
{
inline void setCompletionMapEvidence(msg::FloorCompletion & report, const AuditMapEvidence & evidence)
{
  report.octomap_generation = evidence.version.octomap_generation;
  report.source_octomap_generation = evidence.version.octomap_generation;
  report.floor_map_generation = evidence.version.floor_map_generation;
  report.source_digest = evidence.version.source_digest;
  report.floor_digest = evidence.version.floor_digest;
  report.schema_digest = evidence.schema;
  report.unknown_inventory_digest = evidence.unknowns;
  report.boundary_margin = evidence.margin;
  report.start_x = evidence.start_x;
  report.start_y = evidence.start_y;
  for (const auto & point : evidence.boundary) {
    report.boundary_x.push_back(point.first);
    report.boundary_y.push_back(point.second);
  }
  for (const auto & dependency : evidence.dependencies) {
    msg::FloorMapDependency wire;
    wire.region_id = dependency.region;
    wire.min_x = dependency.min_x;
    wire.min_y = dependency.min_y;
    wire.max_x = dependency.max_x;
    wire.max_y = dependency.max_y;
    wire.floor_digest = dependency.floor_hash;
    wire.octomap_digest = dependency.octomap_hash;
    report.dependencies.push_back(wire);
  }
}

inline std::optional<AuditMapEvidence> completionMapEvidence(const msg::FloorCompletion & m)
{
  if (m.regions.size() > 10000 || !m.octomap_generation || m.octomap_generation != m.source_octomap_generation ||
    !m.floor_map_generation || !m.source_digest || !m.floor_digest || !m.schema_digest ||
    !m.unknown_inventory_digest || m.dependencies.empty() || m.dependencies.size() > 10000 ||
    m.boundary_x.size() != m.boundary_y.size() || m.boundary_x.size() > 10000 ||
    (!m.boundary_x.empty() && m.boundary_x.size() < 3) ||
    !std::isfinite(m.boundary_margin) || m.boundary_margin < 0 ||
    !std::isfinite(m.start_x) || !std::isfinite(m.start_y) || !std::isfinite(m.floor_z)) return {};
  AuditMapEvidence result;
  result.version = {m.octomap_generation, m.floor_map_generation, m.source_digest, m.floor_digest,
    {m.task_id, m.floor_session_id, m.map_producer, m.session_generation}};
  result.floor_z = m.floor_z;
  result.margin = m.boundary_margin;
  result.start_x = m.start_x;
  result.start_y = m.start_y;
  result.schema = m.schema_digest;
  result.unknowns = m.unknown_inventory_digest;
  for (size_t i = 0; i < m.boundary_x.size(); ++i) {
    if (!std::isfinite(m.boundary_x[i]) || !std::isfinite(m.boundary_y[i])) return {};
    result.boundary.emplace_back(m.boundary_x[i], m.boundary_y[i]);
  }
  std::unordered_set<uint64_t> ids;
  for (const auto & dep : m.dependencies) {
    if (!std::isfinite(dep.min_x) || !std::isfinite(dep.min_y) ||
      !std::isfinite(dep.max_x) || !std::isfinite(dep.max_y) || dep.min_x > dep.max_x ||
      dep.min_y > dep.max_y || !dep.floor_digest || !dep.octomap_digest ||
      !ids.insert(dep.region_id).second) return {};
    result.dependencies.push_back({dep.region_id, dep.min_x, dep.min_y,
      dep.max_x, dep.max_y, dep.floor_digest, dep.octomap_digest});
  }
  if (!ids.count(0)) return {};  // Measured start and supporting surface dependency.
  for (const auto & region : m.regions) {
    if (region.dependency_id != region.region_id || !ids.count(region.dependency_id)) return {};
  }
  return result;
}

inline bool completionIdentity(const msg::FloorCompletion & m, const msg::FloorSession & s, int64_t now)
{
  return m.protocol_version == 5 && sessionIdentity(m.report_id) && m.sequence &&
    sameSession(s, m.task_id, m.floor_session_id, m.session_generation) &&
    m.explorer_instance == s.explorer_instance && m.map_producer == s.map_producer &&
    std::isfinite(m.floor_z) && std::abs(m.floor_z - s.floor_z) < 1e-6 &&
    m.generated_at.nanosec < 1000000000 && sessionStamp(m.generated_at) >= sessionStamp(s.started_at) &&
    sessionStamp(m.generated_at) <= now + 100000000;
}

inline std::optional<VerifiedFloorCompletion> verifyFloorCompletion(const msg::FloorCompletion & m)
{
  using Report = msg::FloorCompletion;
  using Region = msg::FloorRegionEvidence;
  if (m.revoked || !m.safe_start || !m.complete_views || !m.complete_routes ||
    m.stable_samples < 3 || !std::isfinite(m.stable_seconds) || m.stable_seconds < 10 ||
    !std::isfinite(m.remaining_area) || !std::isfinite(m.transition_area) ||
    !std::isfinite(m.residual_area) || m.residual_area < 0 || m.transition_area < 0 ||
    m.remaining_area < 0 || std::abs(m.remaining_area - m.residual_area - m.transition_area) > 1e-4 ||
    m.region_count != m.regions.size() || m.region_count > 10000) return {};
  const bool empty = m.completion_status == Report::VERIFIED_COMPLETE &&
    m.remaining_area == 0 && m.residual_area == 0 && m.transition_area == 0 && m.regions.empty();
  if (!empty && !(m.completion_status == Report::VERIFIED_WITH_RESIDUALS &&
    m.remaining_area > 0 && !m.regions.empty())) return {};

  std::unordered_set<uint64_t> ids;
  std::ostringstream records;
  records << std::setprecision(17);
  double residual = 0, transition = 0;
  for (const auto & region : m.regions) {
    if (!std::isfinite(region.area) || region.area <= 0 || !ids.insert(region.region_id).second) return {};
    const bool enclosed = region.classification == Region::VERIFIED_ENCLOSED;
    if (enclosed) residual += region.area;
    else if (region.classification == Region::TRANSITION) transition += region.area;
    else return {};
    if (ids.size() > 1) records << ',';
    records << region.region_id << ':' << region.area << ':' << (enclosed ? "verified_enclosed" : "transition");
  }
  if (!std::isfinite(residual) || !std::isfinite(transition) ||
    std::abs(residual - m.residual_area) > 1e-4 || std::abs(transition - m.transition_area) > 1e-4) return {};
  return VerifiedFloorCompletion{m.floor_z, sessionStamp(m.generated_at) * 1e-9,
    m.remaining_area, m.residual_area, m.transition_area,
    empty ? CompletionStatus::VERIFIED_COMPLETE : CompletionStatus::VERIFIED_WITH_RESIDUALS, records.str()};
}
}  // namespace frontier_exploration_ros2
