all:aruco_test aruco_create_marker
aruco_test:aruco_test.cpp aruco.h aruco.cpp
	g++ aruco_test.cpp aruco.cpp -o aruco_test  -O0 -g3 -lcv -lhighgui  
aruco_create_marker:aruco_create_marker.cpp
	g++ aruco_create_marker.cpp   -o aruco_create_marker   -lcv -lhighgui
aruco_test_gl:aruco_test_gl.cpp aruco.h aruco.cpp
	g++ aruco_test_gl.cpp aruco.cpp -o aruco_test_gl  -O0 -g3 -lcv -lhighgui -lGL -lglut -lGLU
clean:
	rm aruco_test aruco_test_gl aruco_create_marker *.o *~ -f
