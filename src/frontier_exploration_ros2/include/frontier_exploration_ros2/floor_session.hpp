#pragma once

#include "frontier_exploration_ros2/msg/floor_session.hpp"
#include "frontier_exploration_ros2/msg/floor_participant.hpp"
#include "frontier_exploration_ros2/msg/floor_map_snapshot.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <random>
#include <string>

namespace frontier_exploration_ros2
{
inline int64_t sessionStamp(const builtin_interfaces::msg::Time & t)
{
  return int64_t(t.sec) * 1000000000 + t.nanosec;
}

inline std::string newSessionIdentity()
{
  std::random_device random;
  std::array<unsigned char, 16> bytes;
  for (auto & byte : bytes) byte = static_cast<unsigned char>(random());
  bytes[6] = (bytes[6] & 0x0f) | 0x40;
  bytes[8] = (bytes[8] & 0x3f) | 0x80;
  constexpr char hex[] = "0123456789abcdef";
  std::string id;
  for (size_t i = 0; i < bytes.size(); ++i) {
    if (i == 4 || i == 6 || i == 8 || i == 10) id += '-';
    id += hex[bytes[i] >> 4];
    id += hex[bytes[i] & 15];
  }
  return id;
}

inline bool sessionIdentity(const std::string & id)
{
  if (id.size() != 36 || id[14] != '4' || std::string("89ab").find(id[19]) == std::string::npos) return false;
  for (size_t i = 0; i < id.size(); ++i) {
    if (i == 8 || i == 13 || i == 18 || i == 23) {
      if (id[i] != '-') return false;
    } else if (!(id[i] >= '0' && id[i] <= '9') && !(id[i] >= 'a' && id[i] <= 'f')) return false;
  }
  return true;
}

inline bool sameSession(const msg::FloorSession & session, const std::string & task,
  const std::string & floor, uint64_t generation)
{
  return generation && session.generation == generation && session.task_id == task &&
    session.floor_session_id == floor;
}

inline bool validSession(const msg::FloorSession & s, int64_t now)
{
  return sessionIdentity(s.task_id) && sessionIdentity(s.floor_session_id) &&
    sessionIdentity(s.explorer_instance) && sessionIdentity(s.map_producer) &&
    std::isfinite(s.floor_z) && s.generation &&
    s.phase >= msg::FloorSession::PREPARING && s.phase <= msg::FloorSession::CLOSED &&
    sessionStamp(s.started_at) > 0 && s.started_at.nanosec < 1000000000 &&
    s.heartbeat.nanosec < 1000000000 && sessionStamp(s.heartbeat) >= sessionStamp(s.started_at) &&
    sessionStamp(s.heartbeat) <= now + 100000000;
}

// Participants pin their manager's task for the lifetime of the process. A new
// manager cannot acquire those participants while old motion ownership is unknown.
inline bool canApplySession(const msg::FloorSession & current, const msg::FloorSession & next,
  bool idle)
{
  if (!current.generation) return idle && next.phase == msg::FloorSession::PREPARING;
  if (current.task_id != next.task_id || current.explorer_instance != next.explorer_instance ||
    current.map_producer != next.map_producer) return false;
  if (current.generation == next.generation) {
    return current.floor_session_id == next.floor_session_id && current.floor_z == next.floor_z &&
      current.started_at == next.started_at && next.phase >= current.phase &&
      sessionStamp(next.heartbeat) >= sessionStamp(current.heartbeat);
  }
  return idle && current.phase == msg::FloorSession::CLOSED && next.generation == current.generation + 1 &&
    next.floor_session_id != current.floor_session_id && next.phase == msg::FloorSession::PREPARING;
}

inline bool sessionSnapshot(const msg::FloorSession & s, const msg::FloorMapSnapshot & map)
{
  return sameSession(s, map.task_id, map.floor_session_id, map.session_generation) &&
    map.map_producer == s.map_producer && s.phase != msg::FloorSession::CLOSED &&
    std::abs(map.floor_map.info.origin.position.z - s.floor_z) < 1e-6 &&
    sessionStamp(map.octomap.header.stamp) >= sessionStamp(s.started_at) &&
    sessionStamp(map.floor_map.header.stamp) >= sessionStamp(s.started_at);
}

template<typename Control>
void stampSessionControl(Control & request, const msg::FloorSession & session, uint64_t sequence)
{
  request.task_id = session.task_id;
  request.floor_session_id = session.floor_session_id;
  request.session_generation = session.generation;
  request.explorer_instance = session.explorer_instance;
  request.command_sequence = sequence;
}

template<typename Control>
bool sessionControlMatches(const Control & request, const msg::FloorSession & session)
{
  return sameSession(session, request.task_id, request.floor_session_id, request.session_generation) &&
    request.explorer_instance == session.explorer_instance && request.command_sequence;
}
}  // namespace frontier_exploration_ros2
