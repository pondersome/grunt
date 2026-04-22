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

# Launch files — dispatcher + per-mode entry points. Scripts (mode
# switcher node) will get added in Phase C via install(PROGRAMS ...).
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
