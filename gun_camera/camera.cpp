
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

#define SHOW_IMAGE
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
  auto lastMode = devastar::AIM_CALIBRATE_START;
  auto lastButton = devastar::BUTTON_NONE;
  if( argc > 1 ) {
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
  double minBlobSize = 6.0;
  double maxBlobSize = 250.0;

  // from obvservation with delay4Cycles() on arduino
  // Menacer
  // X range is 69 to 263 : send 0 through 194
  // Y range is 26 to 247 : send 0 through 221
  // XG-1
  // X range is 40 to 237 : send 0 through 197
  // Y range is 21 to 210 : send 0 through 189
  // Sega Light Phaser
  // X range is 24 to 247 : send 0 through 223
  // Y range is 15 to 193 : send 0 through 178
  devastar::AimCalibration ac (
      // IR spacing
      510.0f, //  irWidth 
      260.0f, //  irHeight
      // X/Y Values to receiver
      73.0f,  //  outXMin 
      30.0f,  //  outYMin  
      269.0f, //  outXMax 
      250.0f  //  outYMax
      ); 

  if( fs::exists( configPath ) ) {
    ac.readConfig( configPath );
    cv::FileStorage fileStorage(configPath, cv::FileStorage::READ);
    if(!fileStorage["serial_device"].empty() ) {
      fileStorage["serial_device"] >> serialDevice;
    }
    if(!fileStorage["min_blob_size"].empty() ) {
      fileStorage["min_blob_size"] >> minBlobSize;
    }
    if(!fileStorage["max_blob_size"].empty() ) {
      fileStorage["max_blob_size"] >> maxBlobSize;
    }
  }
  devastar::AimCalibrator aimCalibrator(ac, 5, configPath.string() );

  std::cout << "Using: serial device: " << serialDevice << "\n";
  std::cout << " ir_width: " << ac.irWidth << "\n";
  std::cout << " ir_height: " << ac.irHeight << "\n";
  std::cout << " output_x_min: " << ac.outXMin << "\n";
  std::cout << " output_y_min: " << ac.outYMin << "\n";
  std::cout << " output_x_max: " << ac.outXMax << "\n";
  std::cout << " output_y_max: " << ac.outYMax << "\n";
  std::cout << "    out width: " << ac.outWidth << "\n";
  std::cout << "   out height: " << ac.outHeight << "\n";
  std::cout << "        u_min: " << ac.uMin << "\n";
  std::cout << "        v_min: " << ac.vMin << "\n";
  std::cout << "        u_max: " << ac.uMax << "\n";
  std::cout << "        v_max: " << ac.vMax << "\n";
  std::cout << "    U/V width: " << ac.uWidth << "\n";
  std::cout << "   U/V height: " << ac.vHeight << "\n";

  bool use16BitData = ac.outWidth > 255 || ac.outHeight > 255;
  std::cout << " use16BitData: " << use16BitData << "\n";

  // check calibration path
  fs::path calibPath("./calib.yml");
  if( !fs::exists( calibPath ) ) {
    std::cerr << "No camera intrinsics calibration file found.  Please run ./bin/camera_calibrate." << std::endl;
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

  // Set up Serial Configuration
  struct termios serial;
  char buffer[BUFFER_SIZE];
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

  cfmakeraw(&serial);

  serial.c_cflag |= (CLOCAL | CREAD);
  serial.c_iflag &= ~(IXOFF | IXANY);

  serial.c_cc[VMIN] = 0;
  serial.c_cc[VTIME] = 0;

  cfsetispeed(&serial, B9600);
  cfsetospeed(&serial, B9600);

  tcsetattr(fd, TCSANOW, &serial); // Apply configuration	

  // setup image processing variables
  std::vector<cv::Point2f> centers;
  centers.resize(4);
  cv::Mat frame;
  cv::Mat gray;
  cv::Mat thresh;
  cv::Mat displayCopy;

  // setup output
  unsigned char offscreen[] = { 0xFF, 0xFF, 0x00 };
  unsigned char offscreen16[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0x00 };
  unsigned char xyb[3];  // 8 bits for x, y and buttons
  unsigned short sx, sy;
  unsigned char xxyyb[5]; // 16 bits for X and Y outputs. 8 bits for buttons
  unsigned char buttons = 0;

  // look at the cameras
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
    if(err)
    {
      perror("gpiod_line_get_value_bulk");
      gpio_cleanup();
      return -4;
    }

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
      buttons |= devastar::BUTTON_D;   // start button on menacer
    }

    // Process Video from camera
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

    // threshold on brightness
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

#ifdef SHOW_TIME
    startTime = std::chrono::steady_clock::now();
#endif
    float u = -1.0f, v = -1.0f;
    bool solveRet = false;
    // Look for screen intersection if we have at least 4 points
    if (contours.size() >= 4) {
#ifdef SHOW_TIME
      startTime = std::chrono::steady_clock::now();
#endif
      // compute moments to get centers.  
      std::vector< cv::Moments > moments( contours.size() );
      for( size_t i = 0; i < contours.size(); ++i ) {
        moments[i] = cv::moments( contours[i] );
      }

      int centerCount = 0;
      for( size_t i = 0; i < contours.size(); ++i ) {
        // using size cutoff to find 'good' candidates.  Look at other features like circularity, position, etc.
        if( moments[i].m00 > minBlobSize && moments[i].m00 < maxBlobSize ) {

          if( centerCount < 4 ) {
            //centers[centerCount] =  cv::Point2f( static_cast<float> ( moments[i].m10 / ( moments[i].m00 + 1e-5)), static_cast<float> ( moments[i].m01 / ( moments[i].m00 + 1e-5)) );
            centers[centerCount] = cv::Point2f( static_cast<float> ( moments[i].m10 / ( moments[i].m00 + 1e-5)), static_cast<float> ( moments[i].m01 / ( moments[i].m00 + 1e-5)) );
            //#ifdef SHOW_CALC
         
            //std::cout << "center[" << centerCount << "] = " << centers[centerCount] <<  " area: " << moments[i].m00 <<std::endl;
          }
          //#endif
          ++centerCount;
          
          if( centerCount == 4 ) {
            break;
          }
        }
      }
      if( centerCount != 4 ) {
        continue;
      }
#ifdef SHOW_TIME
      endTime = std::chrono::steady_clock::now();
      std::cout << "ELAPSED TIME>> get centers from moments: " << std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() << "\n"; 
#endif


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
      if( centers[a].x < centers[b].x ) {
        ab1 = a;
        ab2 = b;
      } else {
        ab1 = b;
        ab2 = a;
      }

      if( centers[c].x < centers[d].x ) {
        cd1 = c;
        cd2 = d;
      } else {
        cd1 = d;
        cd2 = c;
      }

      if( centers[ab1].x < centers[cd1].x ) {
        i1 = ab1;
        m1 = cd1;
      } else {
        i1 = cd1;
        m1 = ab1;
      }

      if( centers[ab2].x > centers[cd2].x ) {
        i4 = ab2;
        m2 = cd2;
      } else {
        i4 = cd2;
        m2 = ab2;
      }

      if( centers[m1].x < centers[m2].x ) {
        i2 = m1;
        i3 = m2;
      } else {
        i2 = m2;
        i3 = m1;
      }

      cv::Point2f pt1 = centers[i1];
      cv::Point2f pt2 = centers[i2];
      cv::Point2f pt3 = centers[i3];
      cv::Point2f pt4 = centers[i4];

      // compare higehts of first two and last two
      if( pt1.y < pt2.y ) {
        centers[0] = pt1;
        centers[2] = pt2;
      } else {
        centers[0] = pt2;
        centers[2] = pt1;
      }
      if( pt3.y < pt4.y ) {
        centers[1] = pt3;
        centers[3] = pt4;
      } else {
        centers[1] = pt4;
        centers[3] = pt3;
      }

#ifdef SHOW_TIME
      endTime = std::chrono::steady_clock::now();
      std::cout << "ELAPSED TIME>> sort centers: " << std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() << "\n"; 
#endif


#ifdef SHOW_IMAGE
      // display it
      //cv::cvtColor( thresh, displayCopy, cv::COLOR_GRAY2BGR); 
      displayCopy = frame.clone();
      for( int i=0; i < centers.size(); ++ i ) {
        // draw point on image
        cv::Scalar color( 0,0,255 );
        if( i == 1 ) {
          color = cv::Scalar( 0,255,0 );
        } else if( i== 2 ) {
          color = cv::Scalar( 255,0,0 );
        } else if( i== 3 ) {
          color = cv::Scalar( 255,255,0 );
        }
        cv::circle( displayCopy, centers[i], 10.0f, color, 2.0f);
      }
      //cv::circle( displayCopy, pt, 5.0f, cv::Scalar(0,255,255), 3.0f);
      
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
      solveRet = cv::solvePnP(worldPoints, centers, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_AP3P);
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

#ifdef SHOW_TIME
        startTime = std::chrono::steady_clock::now();
#endif
        bool hit = false;
        //hit = intersectRect(Ray0, D, P0, S1, S2, irWidth, irHeight, u, v);
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
        hit = u > ac.uMin && u < ac.uMax && v > ac.vMin && v < ac.vMax;
        if( hit ) {
#ifdef SHOW_IMAGE
          cv::Point2f pt;
          pt.x = (u/ac.irWidth) * 640.0f;
          pt.y = (v/ac.irHeight) * 480.0f;
          cv::circle( displayCopy, pt, 5.0f, cv::Scalar(0,255,255), 3.0f);
          cv::imshow("points", displayCopy );
          cv::waitKey(1);
#endif
          auto outX = ( ( (u-ac.uMin) / ac.uWidth) * ac.outWidth  ) + ac.outXMin;
          auto outY = ( ( (v-ac.vMin) / ac.vHeight) * ac.outHeight + ac.outYMin  );
          if( !use16BitData ) {
            xyb[0] = (unsigned char)outX;
            xyb[1] = (unsigned char)outY;
            xyb[2] = buttons;
            if(serialPortReady ) {
              auto ret = write( fd, xyb, sizeof(xyb) );
            }
          } else {
            sx = (unsigned short)outX;
            sy = (unsigned char)outY;
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



    if( doCalibration ) {
      // get state from aimcalibrator
      auto currentMode = aimCalibrator.getMode();
      auto currentShots = aimCalibrator.getCurrentSampleCount();
      auto maxShots = aimCalibrator.getMaxSamples();

      if( currentMode != lastMode ) {
        switch( currentMode ) {
          case devastar::AIM_CALIBRATE_UPPER_LEFT:
            std::cout << "\33[2K\rAim at upper left corner and pull trigger: " << currentShots << "/" << maxShots << "      " << std::flush;
            // upper left, print out message to aim at upper left pull trigger + count
            break;
          case devastar::AIM_CALIBRATE_LOWER_RIGHT:
            // lower right, print out message to aim at lower right pull trigger + count
            std::cout << "\33[2K\rAim at lower right corner and pull trigger: " << currentShots << "/" << maxShots << "      " << std::flush;
            break;
          case devastar::AIM_CALIBRATE_CALIBRATED:
            std::cout << "\33[2K\rCalibrated: B to restart, C to Cancel, Start to Save          " << std::flush;
            // tell user to press B to redo, C to cancel calibration, S to save
            break;
          case devastar::AIM_CALIBRATE_SAVED:
            std::cout << "\33[2K\rSaved: B to restart calibration, C or Start to exit      " << std::flush;
            // tell user to press B to redo, C to cancel calibration, S to save
            break;
          default:
            break;
        }
        // set the mode
        lastMode = currentMode;

      }


      if( buttons & devastar::BUTTON_A ) {
        if( lastButton != devastar::BUTTON_A ) {
          std::cout << "TRIGGER: " << u << "," << v << std::endl;
          lastButton = devastar::BUTTON_A;
          if( solveRet ) {
            // u and v were calcuatled, pass on to the calibrator
            aimCalibrator.appendSample( u, v );
          }
        } 
      } else if( buttons & devastar::BUTTON_B ) {
        if( lastButton != devastar::BUTTON_B ) {
          std::cout << "Button B" << std::endl;
          lastButton = devastar::BUTTON_B;
          aimCalibrator.restartCalibration();
        } 
      } else if( buttons & devastar::BUTTON_C ) {
        if( lastButton != devastar::BUTTON_C ) {
          std::cout << "Button C" << std::endl;
          lastButton = devastar::BUTTON_C;
          aimCalibrator.cancelCalibration();
        } 
      } else if( buttons & devastar::BUTTON_D ) {
        if( lastButton != devastar::BUTTON_D ) {
          std::cout << "Button D" << std::endl;
          lastButton = devastar::BUTTON_D;
          aimCalibrator.saveCalibration();
        } 
      } else if ( buttons == 0 ) {
        lastButton = devastar::BUTTON_NONE;
      }
    }

    // send -1, -1 to arduino
    if(serialPortReady ) {
      if( !use16BitData ) {
        offscreen[2] = buttons;
		    std::cout << "write() 3: " << int( buttons ) << std::endl;
        auto ret = write( fd, offscreen, sizeof(offscreen) );
      } else {
        offscreen[4] = buttons;
		    std::cout << "write() 4: " << int( buttons ) << std::endl;
        auto ret = write( fd, offscreen16, sizeof(offscreen16) );
      }
    }

  }// while(true)
}
