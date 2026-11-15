# Atomic, immutable floor slice and the exact source used to build it.
# Generations belong to the FloorSliceNode producer; no floor/session protocol.
uint64 octomap_generation
uint64 floor_map_generation
uint64 source_octomap_generation
octomap_msgs/Octomap octomap
nav_msgs/OccupancyGrid floor_map
