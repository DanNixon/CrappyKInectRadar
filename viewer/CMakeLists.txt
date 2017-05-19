cmake_minimum_required(VERSION 2.6)

project(KinectThing)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

set(THREADS_USE_PTHREADS_WIN32 true)

find_package(libusb-1.0 REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(
  ${LIBUSB_1_INCLUDE_DIRS}
  ${THREADS_PTHREADS_INCLUDE_DIR}
  ${OPENGL_INCLUDE_DIR}
  ${GLUT_INCLUDE_DIR}
  serial/include)

add_subdirectory(serial)

add_executable(KinectThing
  main.cpp)

set_property(TARGET KinectThing PROPERTY CXX_STANDARD 11)
set_property(TARGET KinectThing PROPERTY CXX_STANDARD_REQUIRED ON)

target_link_libraries(KinectThing
  serial
  freenect
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARY}
  ${CMAKE_THREAD_LIBS_INIT}
  ${MATH_LIB})
