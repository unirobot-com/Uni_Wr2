include "uni_carto_robot_2d.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 2,
}

POSE_GRAPH.optimize_every_n_nodes = 10
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 7

POSE_GRAPH.global_sampling_ratio = 0.001
POSE_GRAPH.constraint_builder.sampling_ratio = 0.2

POSE_GRAPH.constraint_builder.min_score = 0.6
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.global_constraint_search_after_n_seconds = 10


return options