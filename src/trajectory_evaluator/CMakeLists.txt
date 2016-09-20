set(FILES
  trajectory_sim_grade.cpp
  trajectory_optimizer.cpp
)
add_library( tuw_trajectory_evaluator SHARED ${FILES})
target_link_libraries( tuw_trajectory_evaluator tuw_trajectory_simulator )
