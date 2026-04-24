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

# Python scripts — helper nodes launched alongside or around Nav2:
#   clear_costmap_timer — periodic ClearCostmapAroundRobot caller,
#     used in indoor mode to keep sparse-sensor stale marks from
#     accumulating. See memory/project_costmap_hygiene_backlog.md.
#   nav_mode_switcher — Phase C1 runtime mode switcher. Lives in the
#     main grunt stack (not inside Nav2) so it stays up while Nav2
#     itself is being restarted between modes. Drives systemd
#     grunt-nav@<mode>.service units.
install(PROGRAMS
  scripts/clear_costmap_timer
  scripts/nav_mode_switcher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
