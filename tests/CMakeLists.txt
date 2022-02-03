link_libraries(${OpenCV_LIBS})

# first familiarization with opencv visualization
add_executable(first_drawing_test first_drawing_test.cpp
  ../utils/draw_utils.cpp
)
target_link_libraries(first_drawing_test opencv_highgui)

# testing basic features of state structure, and plotting it
add_executable(state_test state_test.cpp
  ../utils/draw_utils.cpp
  ../framework/state.cpp
)
target_link_libraries(state_test opencv_highgui)
