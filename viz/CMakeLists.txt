# The visualizer links the kinematics library, never the reverse: the core and
# the benchmark harness stay headless.
find_package(SFML 3 COMPONENTS Graphics Window System REQUIRED)

add_executable(arm_viz
  main.cpp
  scene.cpp
  render.cpp
)
target_link_libraries(arm_viz PRIVATE arm_core SFML::Graphics SFML::Window SFML::System)
target_compile_options(arm_viz PRIVATE -Wall -Wextra)
