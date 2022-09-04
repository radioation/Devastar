
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <chrono>

#include <gpiod.h>

#include <experimental/string_view>

#include <iostream>
#include <vector>

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#define BUFFER_SIZE 64
#define BAUDRATE B38400            
#define DEFAULT_SERIAL_DEVICE "/dev/ttyACM0"

//#define SHOW_IMAGE
//#define SHOW_CALC
//#define SHOW_3D
//#define SHOW_TIME


#include <filesystem>
namespace fs = std::filesystem;


#include "aim_calib.h"
#include "devastar_common.h"

#ifdef SHOW_3D
#include <GL/freeglut.h>
void init3d( const std::vector<cv::Point3f>& worldPoints );
void update3d( const cv::Mat& tvec, const cv::Mat& R, const float& u, const float& v  );
#endif



// intersect rectangle with a ray
// * Ray is defined as R0 + t * D
// 
// * Rectangle is represented with a corner point P0
// and two vectors ( S1 and S2 ) indicating the length
// of the sides of the  rectangle.

bool intersectRect(const cv::Vec3f& R0, // ray start
    const cv::Vec3f& D, // ray direction 
    const cv::Vec3f& P0,// Rectangle Origin 
    const cv::Vec3f& S1, // side 1. direction
    const cv::Vec3f& S2, // side 2. direction 
    const float& S1Len,  // side 1. length
    const float& S2Len,  // side 2. length
    float& u,            // u 
    float& v	     // v	
    ) {

  // compute plane Normal 
  cv::Vec3f N = S1.cross(S2);
  float DdotN = D.dot(N);

  // get point on plane P
  auto a = ((P0 - R0).dot(N)) / D.dot(N);
  // we assume that P = R0 + a * D
  cv::Vec3f P = R0 + a * D;  // Point on Plane

  // make vector out of point on plane and orign
  cv::Vec3f P0P = P - P0;

  // project P0P vector onto the sides.
  u = P0P.dot(S1);
  v = P0P.dot(S2);
  return (u >= 0 && u <= S1Len && v >= 0 && v <= S2Len);

}



void computeUV(const cv::Vec3f& R0, // ray start
    const cv::Vec3f& D, // ray direction 
    const cv::Vec3f& P0,// Rectangle Origin 
    const cv::Vec3f& S1, // side 1. direction
    const cv::Vec3f& S2, // side 2. direction 
    const float& S1Len,  // side 1. length
    const float& S2Len,  // side 2. length
    float& u,            // u   // could be 
    float& v	     // v	
    ) {

  // compute plane Normal 
  cv::Vec3f N = S1.cross(S2);
  float DdotN = D.dot(N);

  // get point on plane P
  auto a = ((P0 - R0).dot(N)) / D.dot(N);
  // we assume that P = R0 + a * D
  cv::Vec3f P = R0 + a * D;  // Point on Plane

  // make vector out of point on plane and orign
  cv::Vec3f P0P = P - P0;

  // project P0P vector onto the sides.
  u = P0P.dot(S1);
  v = P0P.dot(S2);
  //return (u >= 0 && u <= S1Len && v >= 0 && v <= S2Len);

}


// Button variables 
struct gpiod_chip * chip;
struct gpiod_line_request_config config;
struct gpiod_line_bulk lines;

void gpio_cleanup() {
  gpiod_line_release_bulk( &lines );
  gpiod_chip_close( chip );
}





int main(int argc, char* argv[] )
{
  bool doCalibration = false;
  auto currentMode = devastar::AIM_CALIBRATE_START;
  auto  lastButton = devastar::BUTTON_NONE;
  if( argc > 0 ) {
    // check argc
    if( std::experimental::string_view( argv[1] ) == "calibrate" ) {
      doCalibration = true;
    } else {
      std::cerr << "Usage: " << argv[0] << " [calibrate]\n";
      return -1;
    }
  }
  // check configuration path
  fs::path configPath("./config.yml");

  std::string serialDevice = DEFAULT_SERIAL_DEVICE;


  // from obvservation with delay4Cycles() on arduino
  // Menacer
  // X range is 73 to 269 : send 0 through 196
  // Y range is 30 to 250 : send 0 through 220
  // XG-1
  // X range is 40 to 237 : send 0 through 197
  // Y range is 21 to 210 : send 0 through 189
  // Sega Light Phaser
  // X range is 24 to 247 : send 0 through 223
  // Y range is 15 to 193 : send 0 through 178
  devastar::AimCalibration ac (
      510.0f, //  irWidth 
      260.0f, //  irHeight
      196.0f, //  outWidth
      220.0f, //  outHeight
      0.0f,   //  outXMin 
      0.0f    //  outYMin
      ); 

  if( fs::exists( configPath ) ) {
    ac.readConfig( configPath );
    cv::FileStorage fileStorage(configPath, cv::FileStorage::READ);
    if(!fileStorage["serial_device"].empty() ) {
      fileStorage["serial_device"] >> serialDevice;
    }
  }
  devastar::AimCalibrator aimCalibrator(ac);

  std::cout << "Using: serial device: " << serialDevice << "\n";
  std::cout << " ir_width: " << ac.irWidth << "\n";
  std::cout << " ir_height: " << ac.irHeight << "\n";
  std::cout << " output_width: " << ac.outWidth << "\n";
  std::cout << " output_height: " << ac.outHeight << "\n";
  std::cout << " output_x_min: " << ac.outXMin << "\n";
  std::cout << " output_y_min: " << ac.outYMin << "\n";

  bool useShortData = ac.outWidth > 255 || ac.outHeight > 255;
  std::cout << " useShortData: " << useShortData << "\n";

  // check calibration path
  fs::path calibPath("./calib.yml");
  if( !fs::exists( calibPath ) ) {
    std::cerr << "No intrinsics calibration file found.  Please run ./bin/camera_calibrate." << std::endl;
    return -1;
  }
  // setup GPIO 
  unsigned int offsets[4];
  int values[4];
  int gpioErr;
  chip = gpiod_chip_open("/dev/gpiochip0");
  if(!chip)
  {
    perror("gpiod_chip_open");
    return -1;
  }

  // setup button pins
  offsets[0] = 6;  // Trigger
  offsets[1] = 5;
  offsets[2] = 13;
  offsets[3] = 19; 
  values[0] = -1;
  values[1] = -1;
  values[2] = -1;
  values[3] = -1;
  auto err = gpiod_chip_get_lines(chip, offsets, 4, &lines);
  if(err)
  {
    perror("gpiod_chip_get_lines");
    gpio_cleanup();
    return -2;
  }

  memset(&config, 0, sizeof(config));
  config.consumer = "devestar";
  config.request_type = GPIOD_LINE_REQUEST_DIRECTION_INPUT;
  config.flags = 0;

  // open lines for input
  err = gpiod_line_request_bulk(&lines, &config, values);
  if(err)
  {
    perror("gpiod_line_request_bulk");
    gpio_cleanup();
    return -3;
  }


  // Read in camera calibration calibration
  cv::FileStorage fileStorage(calibPath, cv::FileStorage::READ);
  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;
  fileStorage["camera_matrix"] >> cameraMatrix;
  fileStorage["dist_coeffs"] >> distCoeffs;


  // setup videocapture
  cv::VideoCapture inputVideo;
  inputVideo.open(0);
  inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 480);


  // Setup object points
  std::vector<cv::Point3f> worldPoints;
  worldPoints.push_back(cv::Point3f(0, 0, 0));
  worldPoints.push_back(cv::Point3f(ac.irWidth, 0, 0));
  worldPoints.push_back(cv::Point3f(0, ac.irHeight, 0));
  worldPoints.push_back(cv::Point3f(ac.irWidth, ac.irHeight, 0));
  // setup rectangle for intersection test
  cv::Vec3f S1, S2, P0;
  P0[0] = worldPoints[0].x;
  P0[1] = worldPoints[0].y;
  P0[2] = worldPoints[0].z;

  S1[0] = worldPoints[1].x - P0[0];
  S1[1] = worldPoints[1].y - P0[1];
  S1[2] = worldPoints[1].z - P0[2];
  S1 = cv::normalize(S1);

  S2[0] = worldPoints[2].x - P0[0];
  S2[1] = worldPoints[2].y - P0[1];
  S2[2] = worldPoints[2].z - P0[2];
  S2 = cv::normalize(S2);

#ifdef SHOW_3D
  glutInit(&argc, argv);
  init3d(worldPoints);
#endif

  // setup serial communication
  struct termios serial;
  char buffer[BUFFER_SIZE];

  // not controlling TTY.  
  bool serialPortReady = true;
  int fd = open( serialDevice.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if( fd < 0 ) {
    std::cerr << "Unable to open serial" << std::endl;
    perror( serialDevice.c_str() );
    serialPortReady = false;
  }
  auto result = tcgetattr( fd, & serial );
  if( result < 0 ) {
    std::cerr << "Unable to get serial attributes" << std::endl;
    serialPortReady = false;
  }

  // Set up Serial Configuration
  cfmakeraw(&serial);

  serial.c_cflag |= (CLOCAL | CREAD);
  serial.c_iflag &= ~(IXOFF | IXANY);

  serial.c_cc[VMIN] = 0;
  serial.c_cc[VTIME] = 0;

  cfsetispeed(&serial, B9600);
  cfsetospeed(&serial, B9600);

  tcsetattr(fd, TCSANOW, &serial); // Apply configuration	

  // setup processing variables
  std::vector<cv::Point2f> centers;
  centers.resize(4);
  cv::Mat frame;
  cv::Mat gray;
  cv::Mat thresh;
  cv::Mat displayCopy;
  unsigned char offscreen[] = { 0xFF, 0xFF, 0x00 };
  unsigned char xyb[3];
  unsigned short sx, sy;
  unsigned char xxyyb[5];
  unsigned char buttons = 0;

  // look at the cmameras
  cv::Point2f pt;
  while (true) {
#ifdef SHOW_3D
    glutMainLoopEvent();
#endif
    // Process buttons
    buttons = 0;
#ifdef SHOW_TIME
    auto startTime = std::chrono::steady_clock::now();
#endif
    err = gpiod_line_get_value_bulk(&lines, values);
#ifdef SHOW_TIME
    auto endTime = std::chrono::steady_clock::now();
    std::cout << "ELAPSED TIME>> frame gpiod_line_get_value_bulk(): " << std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() << "\n"; 
#endif
    if( !values[0] ) {
      buttons |= devastar::BUTTON_A;
    }
    if( !values[1] ) {
      buttons |= devastar::BUTTON_B;
    }
    if( !values[2] ) {
      buttons |= devastar::BUTTON_C;
    }
    if( !values[3] ) {
      buttons |= devastar::BUTTON_D; 
    }
    if(err)
    {
      perror("gpiod_line_get_value_bulk");
      gpio_cleanup();
      return -4;
    }

    // Process Video
#ifdef SHOW_TIME
    startTime = std::chrono::steady_clock::now();
#endif
    inputVideo >> frame;
#ifdef SHOW_TIME
    endTime = std::chrono::steady_clock::now();
    std::cout << "ELAPSED TIME>> frame grab: " << std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() << "\n"; 
#endif
    // cv to grey
#ifdef SHOW_TIME
    startTime = std::chrono::steady_clock::now();
#endif
    cv::cvtColor( frame, gray, cv::COLOR_BGR2GRAY); 
#ifdef SHOW_TIME
    endTime = std::chrono::steady_clock::now();
    std::cout << "ELAPSED TIME>> cvtColor(): " << std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() << "\n"; 
#endif

    // threshold
#ifdef SHOW_TIME
    startTime = std::chrono::steady_clock::now();
#endif
    cv::threshold( gray, thresh, 200, 255, cv::THRESH_BINARY);
#ifdef SHOW_TIME
    endTime = std::chrono::steady_clock::now();
    std::cout << "ELAPSED TIME>> cv::threshold(): " << std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() << "\n"; 
#endif
    std::vector< std::vector< cv::Point> > contours;

    // look for IR lights
#ifdef SHOW_TIME
    startTime = std::chrono::steady_clock::now();
#endif
    cv::findContours( thresh, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
#ifdef SHOW_TIME
    endTime = std::chrono::steady_clock::now();
    std::cout << "ELAPSED TIME>> cv::findContours(): " << std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() << "\n"; 
#endif

    // process if we have at least 4 points
#ifdef SHOW_TIME
    startTime = std::chrono::steady_clock::now();
#endif
    if (contours.size() >= 4) {
#ifdef SHOW_TIME
      startTime = std::chrono::steady_clock::now();
#endif
      // compute moments to get centers.  
      std::vector< cv::Moments > moments( contours.size() );
      for( size_t i = 0; i < contours.size(); ++i ) {
        moments[i] = cv::moments( contours[i] );
      }

      int current = 0;
      for( size_t i = 0; i < contours.size(); ++i ) {
        // using size cutoff to find 'good' candidates.  Look at other features like circularity, position, etc.
        if( moments[i].m00 > 6 && moments[i].m00 < 250 ) {
          centers[current] =  cv::Point2f( static_cast<float> ( moments[i].m10 / ( moments[i].m00 + 1e-5)), static_cast<float> ( moments[i].m01 / ( moments[i].m00 + 1e-5)) );
          //#ifdef SHOW_CALC
          //std::cout << "center[" << current << "] = " << centers[current] <<  " area: " << moments[i].m00 <<std::endl;
          //#endif
          ++current;
          if( current == 4 ) {
            break;
          }
        }
      }
#ifdef SHOW_TIME
      endTime = std::chrono::steady_clock::now();
      std::cout << "ELAPSED TIME>> get centers from moments: " << std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() << "\n"; 
#endif

      //////  REORDER center //////////////////////////////
#ifdef SHOW_TIME
      startTime = std::chrono::steady_clock::now();
#endif
      cv::Point2f a = centers[0];		
      cv::Point2f b = centers[1];		
      if( centers[3].x > centers[2].x ) {
        centers[0] = centers[2];
        centers[1] = centers[3];
      } else {
        centers[0] = centers[3];
        centers[1] = centers[2];
      }
      if( b.x > a.x ) {
        centers[2] = a;
        centers[3] = b;
      } else {
        centers[2] = b;
        centers[3] = a;
      }
#ifdef SHOW_TIME
      endTime = std::chrono::steady_clock::now();
      std::cout << "ELAPSED TIME>> rerder centers: " << std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() << "\n"; 
#endif

#ifdef SHOW_IMAGE
      // display it
      //cv::cvtColor( thresh, displayCopy, cv::COLOR_GRAY2BGR); 
      displayCopy = frame.clone();
      for( int i=0; i < centers.size(); ++ i ) {
        // draw point on image
        cv::Scalar color( 0,0,255 );
        if( i > 1 ) {
          // Green are the later two
          color = cv::Scalar( 0,255,0 );
          cv::circle( displayCopy, centers[i], 2.0f, color, 2.0f);
        } else {
          // RED
          cv::circle( displayCopy, centers[i], 10.0f, color, 2.0f);
        }
      }
      cv::circle( displayCopy, pt, 5.0f, cv::Scalar(0,255,255), 3.0f);

      cv::imshow("points", displayCopy );
      cv::waitKey(1);
#endif

      // rvec- is the rotation vector
      // tvec- is the translation vector 
      cv::Mat rvec, tvec;
      std::vector< cv::Mat > rvecs, tvecs;	
#ifdef SHOW_TIME
      startTime = std::chrono::steady_clock::now();
#endif
      auto solveRet = cv::solvePnP(worldPoints, centers, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_AP3P);
#ifdef SHOW_TIME
      endTime = std::chrono::steady_clock::now();
      std::cout << "ELAPSED TIME>> solvePnP(): " << std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() << "\n"; 
#endif
      if( solveRet ) {

        cv::Mat R;
        cv::Rodrigues(rvec, R); // get rotation matrix R ( 3x3 ) from rotation vector 
        R = R.t(); // inverse
        tvec = -R * tvec; // translation of inverseA == actual camera position

        // compute itersection
        cv::Vec3f Ray0,D;
        Ray0[0] = tvec.at<double>(0);
        Ray0[1] = tvec.at<double>(1);
        Ray0[2] = tvec.at<double>(2);

        D[0] = R.at<double>(0, 2);
        D[1] = R.at<double>(1, 2);
        D[2] = R.at<double>(2, 2);

        float u, v;
        //bool hit = false;
#ifdef SHOW_TIME
        startTime = std::chrono::steady_clock::now();
#endif
        //bool hit = intersectRect(Ray0, D, P0, S1, S2, irWidth, irHeight, u, v);
        bool hit = false;
        computeUV(Ray0, D, P0, S1, S2, ac.irWidth, ac.irHeight, u, v);
#ifdef SHOW_TIME
        endTime = std::chrono::steady_clock::now();
        std::cout << "ELAPSED TIME>> intersectRect(): " << std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() << "\n"; 
#endif


#ifdef SHOW_CALC
        std::cout << "rvec: " << rvec << std::endl;	
        std::cout << "tvec: " << tvec << std::endl;
        std::cout << "R: " << R << std::endl;
        std::cout << "inverse tvec: " << tvec << std::endl;

        std::cout << "Ray0: " << Ray0 << std::endl;
        std::cout << "D: " << D << std::endl;
        std::cout << "P0: " << P0 << std::endl;
        std::cout << "S1: " << S1 << std::endl;
        std::cout << "S2: " << S2 << std::endl;
        std::cout << "irWidth: " << ac.irWidth  << std::endl;
        std::cout << "irHeight: " << ac.irHeight << std::endl;
        std::cout << "U: " << u << " V: " << v << " hit: " << hit << std::endl;
#endif



#ifdef SHOW_3D
        update3d( tvec, R, u, v );
#endif

        if( doCalibration ) {
	  // pass U/V to it?
	  if( buttons & devastar::BUTTON_A ) {
		  if( lastButton != devastar::BUTTON_A ) {
			  std::cout << "TRIGGER: " << u << "," << v << std::endl;
			  lastButton = devastar::BUTTON_A;
		  } 
	  } else if( buttons & devastar::BUTTON_B ) {
		  if( lastButton != devastar::BUTTON_B ) {
			  std::cout << "Button B" << std::endl;
			  lastButton = devastar::BUTTON_B;
		  } 
	  } else if( buttons & devastar::BUTTON_C ) {
		  if( lastButton != devastar::BUTTON_C ) {
			  std::cout << "Button C" << std::endl;
			  lastButton = devastar::BUTTON_C;
		  } 
	  } else if ( buttons == 0 ) {
  		lastButton = devastar::BUTTON_NONE;
	  }
        }else if( hit ) {
#ifdef SHOW_IMAGE
          cv::Point2f pt;
          pt.x = (u/ac.irWidth) * 640.0f;
          pt.y = (v/ac.irHeight) * 480.0f;
          cv::circle( displayCopy, pt, 5.0f, cv::Scalar(0,255,255), 3.0f);
          cv::imshow("points", displayCopy );
          cv::waitKey(1);
#endif
          if( !useShortData ) {
            xyb[0] = (unsigned char)( ( u / ac.irWidth) * ac.outWidth  );
            xyb[1] = (unsigned char)( ( v / ac.irHeight) * ac.outHeight  );
            xyb[2] = buttons;
            if(serialPortReady ) {
              auto ret = write( fd, xyb, sizeof(xyb) );
            }
          } else {
            sx = (unsigned short)( ( u / ac.irWidth) * ac.outWidth  );
            sy = (unsigned char)( ( v / ac.irHeight) * ac.outHeight  );
            memcpy( xxyyb, &sx, sizeof( unsigned short ) ); 
            memcpy( xxyyb + sizeof(unsigned short) , &sy, sizeof( unsigned short ) ); 
            xxyyb[4] = buttons;
            if(serialPortReady ) {
              auto ret = write( fd, xyb, sizeof(xyb) );
            }
          }
          continue;  // head back up the loop
        } // if( hit ) 
      } // if( solveRet ) 


    } // if (contours.size() >= 4) 

    // send -1, -1 to arduino
    if(serialPortReady ) {
      if( !useShortData ) {
        offscreen[2] = buttons;
        auto ret = write( fd, offscreen, sizeof(offscreen) );
      } else {
      }
    }

  }// while(true)
}
