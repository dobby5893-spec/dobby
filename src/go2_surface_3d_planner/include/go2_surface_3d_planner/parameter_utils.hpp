#pragma once

#include <string>

namespace go2_surface_3d_planner
{
// Lifecycle cleanup does not undeclare node parameters. Preserve existing
// overrides when plugins are initialized again after a reset.
template<typename T, typename NodePtr>
T getOrDeclareParameter(NodePtr node, const std::string & name, const T & fallback)
{
  if(!node->has_parameter(name)) {node->template declare_parameter<T>(name,fallback);}
  T value=fallback;
  node->get_parameter(name,value);
  return value;
}
}  // namespace go2_surface_3d_planner
