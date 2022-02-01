link_libraries(${OpenCV_LIBS})

# first familiarization with opencv visualization
add_executable(first_drawing_test first_drawing_test.cpp
  ../utils/draw_utils.cpp
  # altri file .cpp
  # altri file .cpp
)
target_link_libraries(first_drawing_test opencv_highgui)
