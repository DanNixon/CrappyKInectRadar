cmake_minimum_required(VERSION 3.1)

project(KinectThing)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

set(THREADS_USE_PTHREADS_WIN32 true)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(Boost 1.45.0 REQUIRED COMPONENTS system program_options)
find_package(libusb-1.0 REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(
  ${Boost_INCLUDE_DIRS}
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
  ${Boost_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARY}
  ${CMAKE_THREAD_LIBS_INIT}
  ${MATH_LIB})
