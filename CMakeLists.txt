cmake_minimum_required(VERSION 3.7)

project(KinectThing)

set(THREADS_USE_PTHREADS_WIN32 true)
set(MATH_LIB "m")

find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(${THREADS_PTHREADS_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})

add_executable(KinectThing main.c)
target_link_libraries(KinectThing freenect ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIB})
