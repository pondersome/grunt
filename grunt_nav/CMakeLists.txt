cmake_minimum_required(VERSION 3.8)
project(grunt_nav)

find_package(ament_cmake REQUIRED)

# Configs — YAMLs consumed by nav_*.launch.py at runtime. Also includes
# future behavior tree XMLs for custom BTs that diverge from Jazzy's
# defaults (indoor and outdoor flavors).
install(DIRECTORY
  config
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

# Launch files — dispatcher + per-mode entry points.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Python scripts — helper nodes launched alongside Nav2. For now:
#   clear_costmap_timer — periodic ClearCostmapAroundRobot caller,
#     used in indoor mode to keep sparse-sensor stale marks from
#     accumulating. See memory/project_costmap_hygiene_backlog.md.
install(PROGRAMS
  scripts/clear_costmap_timer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
