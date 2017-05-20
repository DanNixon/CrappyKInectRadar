/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

#include <cmath>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include <boost/program_options.hpp>
#include <libfreenect.hpp>
#include <serial/serial.h>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif

namespace po = boost::program_options;

class StepperController
{
public:
  enum SweepState
  {
    SWEEP_OFF,
    SWEEP_THERE,
    SWEEP_BACK
  };

public:
  StepperController(serial::Serial &port)
      : m_port(port)
      , m_sweepWidth(500)
      , m_inMotion(false)
  {
    if (!port.isOpen())
      throw std::runtime_error("Serial port is not open");
  }

  void move(int dist)
  {
    std::string message = std::to_string(dist) + "\n";
    m_port.write(message);
  }

  void startAsyncSweep()
  {
    m_sweepState = SWEEP_THERE;
  }

private:
  void workerThreadFunc()
  {
    /* TODO */
  }

private:
  serial::Serial &m_port;
  int m_sweepWidth;
  bool m_inMotion;
  SweepState m_sweepState;
};

class MyFreenectDevice : public Freenect::FreenectDevice
{
public:
  MyFreenectDevice(freenect_context *_ctx, int _index)
      : Freenect::FreenectDevice(_ctx, _index)
      , m_depthBuffer(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes)
      , m_videoBuffer(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes)
      , m_gamma(2048)
      , m_newDepthFrame(false)
      , m_newVideoFrame(false)
  {
    for (unsigned int i = 0; i < 2048; i++)
    {
      float v = i / 2048.0;
      v = std::pow(v, 3) * 6;
      m_gamma[i] = v * 6 * 256;
    }
  }

  void VideoCallback(void *_rgb, uint32_t timestamp)
  {
    std::lock_guard<std::mutex> lock(m_videoMutex);
    uint8_t *rgb = static_cast<uint8_t *>(_rgb);
    std::copy(rgb, rgb + getVideoBufferSize(), m_videoBuffer.begin());
    m_newVideoFrame = true;
  };

  void DepthCallback(void *_depth, uint32_t timestamp)
  {
    std::lock_guard<std::mutex> lock(m_depthMutex);
    uint16_t *depth = static_cast<uint16_t *>(_depth);
    for (unsigned int i = 0; i < 640 * 480; i++)
    {
      int pval = m_gamma[depth[i]];
      int lb = pval & 0xff;
      switch (pval >> 8)
      {
      case 0:
        m_depthBuffer[3 * i + 0] = 255;
        m_depthBuffer[3 * i + 1] = 255 - lb;
        m_depthBuffer[3 * i + 2] = 255 - lb;
        break;
      case 1:
        m_depthBuffer[3 * i + 0] = 255;
        m_depthBuffer[3 * i + 1] = lb;
        m_depthBuffer[3 * i + 2] = 0;
        break;
      case 2:
        m_depthBuffer[3 * i + 0] = 255 - lb;
        m_depthBuffer[3 * i + 1] = 255;
        m_depthBuffer[3 * i + 2] = 0;
        break;
      case 3:
        m_depthBuffer[3 * i + 0] = 0;
        m_depthBuffer[3 * i + 1] = 255;
        m_depthBuffer[3 * i + 2] = lb;
        break;
      case 4:
        m_depthBuffer[3 * i + 0] = 0;
        m_depthBuffer[3 * i + 1] = 255 - lb;
        m_depthBuffer[3 * i + 2] = 255;
        break;
      case 5:
        m_depthBuffer[3 * i + 0] = 0;
        m_depthBuffer[3 * i + 1] = 0;
        m_depthBuffer[3 * i + 2] = 255 - lb;
        break;
      default:
        m_depthBuffer[3 * i + 0] = 0;
        m_depthBuffer[3 * i + 1] = 0;
        m_depthBuffer[3 * i + 2] = 0;
        break;
      }
    }
    m_newDepthFrame = true;
  }

  bool getRGB(std::vector<uint8_t> &buffer)
  {
    std::lock_guard<std::mutex> lock(m_videoMutex);
    if (!m_newVideoFrame)
      return false;
    buffer.swap(m_videoBuffer);
    m_newVideoFrame = false;
    return true;
  }

  bool getDepth(std::vector<uint8_t> &buffer)
  {
    std::lock_guard<std::mutex> lock(m_depthMutex);
    if (!m_newDepthFrame)
      return false;
    buffer.swap(m_depthBuffer);
    m_newDepthFrame = false;
    return true;
  }

private:
  std::vector<uint8_t> m_depthBuffer;
  std::vector<uint8_t> m_videoBuffer;
  std::vector<uint16_t> m_gamma;
  std::mutex m_depthMutex;
  std::mutex m_videoMutex;
  bool m_newDepthFrame;
  bool m_newVideoFrame;
};

enum Mode
{
  MODE_DEPTH,
  MODE_RGB,
  MODE_IR,

  MODE_COUNT
};

Mode mode;

int window = 0;
GLuint imageTexture;
Freenect::Freenect freenect;
MyFreenectDevice *device;
float titleAngle = 0;

StepperController *stepper;

template <typename T> void clamp(T &v, T lo, T hi)
{
  v = std::min(std::max(v, lo), hi);
}

void setDeviceFormatForMode(Mode m)
{
  switch (m)
  {
  case MODE_RGB:
    device->setVideoFormat(FREENECT_VIDEO_RGB);
    break;
  case MODE_IR:
    device->setVideoFormat(FREENECT_VIDEO_IR_8BIT);
    break;
  default:
    break;
  }
}

void drawScene()
{
  static std::vector<uint8_t> buff(640 * 480 * 4);

  device->updateState();

  switch (mode)
  {
  case MODE_DEPTH:
    device->getDepth(buff);
    break;
  case MODE_RGB:
  case MODE_IR:
    device->getRGB(buff);
    break;
  default:
    return;
  }

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

  glEnable(GL_TEXTURE_2D);

  glBindTexture(GL_TEXTURE_2D, imageTexture);
  if (mode == MODE_IR)
    glTexImage2D(GL_TEXTURE_2D, 0, 1, 640, 480, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, &buff[0]);
  else
    glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, &buff[0]);

  glBegin(GL_TRIANGLE_FAN);
  glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
  glTexCoord2f(0, 0);
  glVertex3f(0, 0, 0);
  glTexCoord2f(1, 0);
  glVertex3f(640, 0, 0);
  glTexCoord2f(1, 1);
  glVertex3f(640, 480, 0);
  glTexCoord2f(0, 1);
  glVertex3f(0, 480, 0);
  glEnd();

  glutSwapBuffers();
}

void printInfo()
{
  std::cout << "Available Controls\n";
  std::cout << "==================\n";
  std::cout << "Tilt/Pitch   :   W / S\n";
  std::cout << "Rotate/Yaw   :   A / D\n";
  std::cout << "Set LED mode :   0 - 5\n";
  std::cout << "Toggle mode  :   M\n";
  std::cout << "Quit         :   Q or Esc\n";
}

void keyPressed(unsigned char key, int x, int y)
{
  switch (key)
  {
  case '1':
    device->setLed(LED_GREEN);
    break;

  case '2':
    device->setLed(LED_RED);
    break;

  case '3':
    device->setLed(LED_YELLOW);
    break;

  case '4':
    device->setLed(LED_BLINK_GREEN);
    break;

  case '5':
    device->setLed(LED_BLINK_RED_YELLOW);
    break;

  case '0':
    device->setLed(LED_OFF);
    break;

  case 'A':
  case 'a':
    if (stepper)
      stepper->move(-50);
    break;

  case 'D':
  case 'd':
    if (stepper)
      stepper->move(50);
    break;

  case 'W':
  case 'w':
    titleAngle += 1.0f;
    clamp(titleAngle, -30.0f, 30.0f);
    device->setTiltDegrees(titleAngle);
    break;

  case 'S':
  case 's':
    titleAngle -= 1.0f;
    clamp(titleAngle, -30.0f, 30.0f);
    device->setTiltDegrees(titleAngle);
    break;

  case 'M':
  case 'm':
    mode = (Mode)(mode + 1);
    if (mode == MODE_COUNT)
      mode = MODE_DEPTH;
    setDeviceFormatForMode(mode);
    break;

  case 'H':
  case 'h':
    printInfo();
    break;

  case 'Q':
  case 'q':
  case 0x1B: // ESC
    glutDestroyWindow(window);
    device->stopDepth();
    device->stopVideo();
    exit(0);
  }
}

void rezizeWindow(int width, int height)
{
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, width, height, 0, -1.0f, 1.0f);
  glMatrixMode(GL_MODELVIEW);
}

void initGL(int width, int height)
{
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

  glClearDepth(1.0);
  glDepthFunc(GL_LESS);
  glDisable(GL_DEPTH_TEST);

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glShadeModel(GL_SMOOTH);

  glGenTextures(1, &imageTexture);
  glBindTexture(GL_TEXTURE_2D, imageTexture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  rezizeWindow(width, height);
}

int main(int argc, char **argv)
{
  po::options_description desc("Allowed options");
  // clang-format off
  desc.add_options()("help", "produce help message")(
      "port", po::value<std::string>(), "serial port for stepper driver")("baud", po::value<int>()->default_value(9600),
                                                                          "baud rate for stepper driver");
  // clang-format on

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  if (vm.count("help"))
  {
    std::cout << desc << "\n";
    return 0;
  }

  device = &freenect.createDevice<MyFreenectDevice>(0);
  device->startVideo();
  device->startDepth();

  printInfo();

  serial::Serial port(vm["port"].as<std::string>(), vm["baud"].as<int>());
  stepper = new StepperController(port);

  glutInit(&argc, argv);

  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
  glutInitWindowSize(640, 480);
  glutInitWindowPosition(0, 0);

  window = glutCreateWindow("Crappy Kinect Radar");

  glutDisplayFunc(&drawScene);
  glutIdleFunc(&drawScene);
  glutReshapeFunc(&rezizeWindow);
  glutKeyboardFunc(&keyPressed);

  initGL(640, 480);

  /* Set defaults */
  mode = MODE_RGB;
  setDeviceFormatForMode(mode);
  device->setTiltDegrees(titleAngle);

  glutMainLoop();

  device->stopVideo();
  device->stopDepth();

  return 0;
}
