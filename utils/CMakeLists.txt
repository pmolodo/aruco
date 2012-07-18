INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src ${GNULIBS_INCLUDE_DIR})
LINK_LIBRARIES(${PROJECT_NAME})

ADD_EXECUTABLE(aruco_create_board aruco_create_board.cpp)
ADD_EXECUTABLE(aruco_create_marker aruco_create_marker.cpp)
ADD_EXECUTABLE(aruco_test_board aruco_test_board.cpp)
ADD_EXECUTABLE(aruco_test aruco_test.cpp)

INSTALL(TARGETS aruco_create_board aruco_create_marker aruco_test_board aruco_test RUNTIME DESTINATION bin)


IF(GL_FOUND)
  ADD_EXECUTABLE(aruco_test_gl aruco_test_gl.cpp)
  TARGET_LINK_LIBRARIES(aruco_test_gl ${OPENGL_LIBS})

  ADD_EXECUTABLE(aruco_test_board_gl aruco_test_board_gl.cpp)
  TARGET_LINK_LIBRARIES(aruco_test_board_gl ${OPENGL_LIBS})

  INSTALL(TARGETS aruco_test_gl aruco_test_board_gl RUNTIME DESTINATION bin)
ENDIF(GL_FOUND)
