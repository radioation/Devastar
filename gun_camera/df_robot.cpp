
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <chrono>
#include <bitset>

#include <iostream>
#include <vector>

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include "compute.h"
#include "config.h"
#include "devastar_common.h"
#include "df_robot.h"

#include <filesystem>
namespace fs = std::filesystem;


using namespace devastar;


DFRobot::DFRobot() : m_isRunning(false)
{
}

bool DFRobot::init(const Configuration& conf )
{
  m_conf = conf;
  m_deviceName = m_conf.i2cDevice;
  m_devFile = open( m_deviceName.c_str(), O_RDWR);
  if( m_devFile < 0 ) {
    std::cerr << "Failed to open DFRobot device: " << m_deviceName << std::endl;
    return false;
  }
  int addr = 0x58; // I2C  device address
  if( ioctl(m_devFile, I2C_SLAVE, addr) < 0 ) {
    std::cerr << "Failed to access DFRobot device: " << std::hex << addr << std::endl;
    return false;
  }

  // IR Sensor initialization sequence from DFRobot arduino example
  struct timespec delay;
  struct timespec remains;
  delay.tv_sec = 0;
  delay.tv_nsec = 10000000; // 10 miliseconds
  uint8_t msg1[] = {0x30,0x01};
  uint8_t msg2[] = {0x30,0x08};
  uint8_t msg3[] = {0x06,0x90};
  uint8_t msg4[] = {0x08,0xC0};
  uint8_t msg5[] = {0x1A,0x40};
  uint8_t msg6[] = {0x33,0x33};
  write(m_devFile, msg1, 2); nanosleep(&delay, &remains);
  write(m_devFile, msg2, 2); nanosleep(&delay, &remains);
  write(m_devFile, msg3, 2); nanosleep(&delay, &remains);
  write(m_devFile, msg4, 2); nanosleep(&delay, &remains);
  write(m_devFile, msg5, 2); nanosleep(&delay, &remains);
  write(m_devFile, msg6, 2); nanosleep(&delay, &remains);
  delay.tv_nsec = 5000000; // 5 miliseconds
  nanosleep(&delay, &remains);

  // start the thread


  return true;
}


void DFRobot::getCenters( std::vector<cv::Point2f>& centers ) const {
	std::transform(m_centers.begin(), m_centers.end(), std::back_inserter(centers), 
			[](auto e){ return e; });   
}

bool DFRobot::stop() {
  if( m_isRunning ) {
    m_isRunning = false;
    if( m_captureThread.joinable() ) {
      m_captureThread.join();
    }
    return true;
  }
  return false;
}


void DFRobot::captureThread() {
  m_isRunning = true;
  // setup vars
  m_centers.resize(4);

  char outBuffer[255] = {0}; 
  outBuffer[0] = 0x36; // IR sensor read command
  char readBuffer[255] = {0};
  int Ix[4];
  int Iy[4];
  int s;

#ifdef SHOW_IMAGE
  cv::namedWindow("IR", cv::WINDOW_AUTOSIZE);
  cv::Mat blankImg( 1024, 768, CV_8UC3, cv::Scalar(0,0,0));
  cv::Mat displayImg( 1024, 768, CV_8UC3, cv::Scalar(0,0,0));
#endif


  cv::Point2f pt;
  while (m_isRunning) {


#ifdef SHOW_TIME
    auto startTime = std::chrono::steady_clock::now();
#endif
    // send IR sensor read command
    const size_t outLen = 1;
    if (write(m_devFile, outBuffer, outLen) != outLen)
    {
      std::cerr << "Failed to write to the i2c bus.\n";
    }

    // read 16 bytes over i2C
    const size_t inLen = 16;
    if (read(m_devFile, readBuffer, inLen) != inLen)
    {
      std::cerr << "Failed to read from the i2c bus.\n";
    }
#ifdef SHOW_TIME
    auto endTime = std::chrono::steady_clock::now();
    std::cout << "ELAPSED TIME>> read data: " << float(std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count()) / 1000.0f << "\n"; 
#endif

#ifdef SHOW_TIME
    startTime = std::chrono::steady_clock::now();
#endif
    Ix[0] = readBuffer[1];
    Iy[0] = readBuffer[2];
    s   = readBuffer[3];
    Ix[0] += (s & 0x30) <<4;
    Iy[0] += (s & 0xC0) <<2;

    Ix[1] = readBuffer[4];
    Iy[1] = readBuffer[5];
    s   = readBuffer[6];
    Ix[1] += (s & 0x30) <<4;
    Iy[1] += (s & 0xC0) <<2;

    Ix[2] = readBuffer[7];
    Iy[2] = readBuffer[8];
    s   = readBuffer[9];
    Ix[2] += (s & 0x30) <<4;
    Iy[2] += (s & 0xC0) <<2;

    Ix[3] = readBuffer[10];
    Iy[3] = readBuffer[11];
    s   = readBuffer[12];
    Ix[3] += (s & 0x30) <<4;
    Iy[3] += (s & 0xC0) <<2;

#ifdef SHOW_TIME
    endTime = std::chrono::steady_clock::now();
    std::cout << "ELAPSED TIME>> unpack data: " << float(std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count()) / 1000.0f << "\n"; 
#endif

#ifdef SHOW_IMAGE
    displayImg = blankImg.clone();
    for(int i=0; i<4; i++)
    {
      cv::Point2f pt( Ix[i], Iy[i] );
      cv::Scalar color( 0,0,255);
      if( i == 1 ) {
        color = cv::Scalar( 0,255, 0);
      } else if (i == 2 ) {
        color = cv::Scalar(255,0, 0);
      } else if( i == 3 ) {
        color = cv::Scalar(255,255, 0);
      }
      cv::circle( displayImg, pt, 1.0, color, 2.0f );
    }
    // slight delay
    cv::imshow("IR", displayImg);

#endif


    // order centers for later procesing
    // use simplest case for now -> only calculate hit if count is 4 
    // TODO: Also check if "n >= 4".  potentially look at some distance
    // minimizaion check.
#ifdef SHOW_TIME
    startTime = std::chrono::steady_clock::now();
#endif
    size_t a=0;
    size_t b=1;
    size_t c=2;
    size_t d=3;
    size_t ab1, ab2, cd1,cd2;
    size_t i1, i2, i3, i4, m1, m2;

    // sort along X
    // split into 2 sets {a,b}  and {c,d}
    if( m_centers[a].x < m_centers[b].x ) {
      ab1 = a;
      ab2 = b;
    } else {
      ab1 = b;
      ab2 = a;
    }

    if( m_centers[c].x < m_centers[d].x ) {
      cd1 = c;
      cd2 = d;
    } else {
      cd1 = d;
      cd2 = c;
    }

    if( m_centers[ab1].x < m_centers[cd1].x ) {
      i1 = ab1;
      m1 = cd1;
    } else {
      i1 = cd1;
      m1 = ab1;
    }

    if( m_centers[ab2].x > m_centers[cd2].x ) {
      i4 = ab2;
      m2 = cd2;
    } else {
      i4 = cd2;
      m2 = ab2;
    }

    if( m_centers[m1].x < m_centers[m2].x ) {
      i2 = m1;
      i3 = m2;
    } else {
      i2 = m2;
      i3 = m1;
    }

    cv::Point2f pt1 = m_centers[i1];
    cv::Point2f pt2 = m_centers[i2];
    cv::Point2f pt3 = m_centers[i3];
    cv::Point2f pt4 = m_centers[i4];

    // compare higehts of first two and last two
    if( pt1.y < pt2.y ) {
      m_centers[0] = pt1;
      m_centers[2] = pt2;
    } else {
      m_centers[0] = pt2;
      m_centers[2] = pt1;
    }
    if( pt3.y < pt4.y ) {
      m_centers[1] = pt3;
      m_centers[3] = pt4;
    } else {
      m_centers[1] = pt4;
      m_centers[3] = pt3;
    }

#ifdef SHOW_TIME
    endTime = std::chrono::steady_clock::now();
    std::cout << "ELAPSED TIME>> sort centers: " << float(std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count()) / 1000.0f << "\n"; 
#endif

  }// while(true)
  cv::destroyWindow("IR");
}

