link_libraries(${OpenCV_LIBS})

add_executable(bearing_only_slam bearing_only_slam.cpp
  ../utils/draw_utils.cpp
  ../framework/state.cpp
  ../framework/observation.cpp
  ../utils/g2o_utils.cpp
  ../slam/triangulation.cpp
  ../slam/solver.cpp
  ../slam/solver_jacobians.cpp
)
target_link_libraries(bearing_only_slam opencv_highgui)
