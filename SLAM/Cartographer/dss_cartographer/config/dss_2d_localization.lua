-- Cartographer 2D Localization Configuration for DSS Platform
-- Pure localization mode (no new map building)

include "dss_2d.lua"

-- Disable map building for pure localization
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- Faster pose graph for localization
POSE_GRAPH.optimize_every_n_nodes = 20

return options
