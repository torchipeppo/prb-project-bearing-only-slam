link_libraries(${OpenCV_LIBS})

# first familiarization with opencv visualization
add_executable(first_drawing_test first_drawing_test.cpp
  ../utils/draw_utils.cpp
  ../framework/state.cpp
  ../framework/observation.cpp
)
target_link_libraries(first_drawing_test opencv_highgui)

# testing basic features of state structure, and plotting it
add_executable(state_test state_test.cpp
  ../utils/draw_utils.cpp
  ../framework/state.cpp
  ../framework/observation.cpp
)
target_link_libraries(state_test opencv_highgui)

# testing basic features of observation structure, and plotting it
add_executable(observation_test observation_test.cpp
  ../utils/draw_utils.cpp
  ../framework/state.cpp
  ../framework/observation.cpp
)
target_link_libraries(observation_test opencv_highgui)

# parsing the dataset (an plotting it in all its glory, why not)
add_executable(plot_g2o plot_g2o.cpp
  ../utils/draw_utils.cpp
  ../framework/state.cpp
  ../framework/observation.cpp
  ../utils/g2o_utils.cpp
)
target_link_libraries(plot_g2o opencv_highgui)

# testing the triangulation part, for the initial guess of the landmarks
add_executable(triangulation_test triangulation_test.cpp
  ../utils/draw_utils.cpp
  ../framework/state.cpp
  ../framework/observation.cpp
  ../utils/g2o_utils.cpp
  ../slam/triangulation.cpp
)
target_link_libraries(triangulation_test opencv_highgui)

# Test some things of the solver
add_executable(solver_stuff solver_stuff.cpp
  ../utils/draw_utils.cpp
  ../framework/state.cpp
  ../framework/observation.cpp
  ../utils/g2o_utils.cpp
  ../slam/triangulation.cpp
  ../slam/solver.cpp
  ../slam/solver_jacobians.cpp
)
target_link_libraries(solver_stuff opencv_highgui)

# testing that the addition of the odometry edges went alright
add_executable(plot_g2o_v2 plot_g2o_v2.cpp
  ../utils/draw_utils.cpp
  ../framework/state.cpp
  ../framework/observation.cpp
  ../utils/g2o_utils.cpp
  ../slam/triangulation.cpp
)
target_link_libraries(plot_g2o_v2 opencv_highgui)

# General test of the full procedure. Interactive.
add_executable(testone testone.cpp
  ../utils/draw_utils.cpp
  ../framework/state.cpp
  ../framework/observation.cpp
  ../utils/g2o_utils.cpp
  ../slam/triangulation.cpp
  ../slam/solver.cpp
  ../slam/solver_jacobians.cpp
)
target_link_libraries(testone opencv_highgui)

# Interactive test with only the noisy dataset, as a prelude to the final executable
add_executable(testone_ig testone_ig.cpp
  ../utils/draw_utils.cpp
  ../framework/state.cpp
  ../framework/observation.cpp
  ../utils/g2o_utils.cpp
  ../slam/triangulation.cpp
  ../slam/solver.cpp
  ../slam/solver_jacobians.cpp
)
target_link_libraries(testone_ig opencv_highgui)
