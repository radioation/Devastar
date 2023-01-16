
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <chrono>
#include <bitset>
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

//#define SHOW_IMAGE
//#define SHOW_3D
//#define SHOW_CALC
//#define SHOW_TIME


#include <filesystem>
namespace fs = std::filesystem;

#include "compute.h"
#include "config.h"
#include "aim_calib.h"
#include "devastar_common.h"

#include "ir_camera.h"
#include "df_robot.h"


#ifdef SHOW_3D
#include <GL/freeglut.h>
void init3d( const std::vector<cv::Point3f>& worldPoints );
void update3d( const cv::Mat& tvec, const cv::Mat& R, const float& u, const float& v  );
#endif





// Button variables 
struct gpiod_chip * g_chip;
struct gpiod_line_request_config g_config;
struct gpiod_line_bulk g_lines;

void gpio_cleanup() {
  gpiod_line_release_bulk( &g_lines );
  gpiod_chip_close( g_chip );
}


static void usage(const std::string& programName)
{
  std::cerr << "Usage: " << programName << " <options>"
    << "Options:\n"
    << "  -h,--help        Shows this help message\n"
    << "  -c,--calibrate   Run aim calibrate mode\n"
    << "  -o,--config_file Specify configuration file [config.yml]\n"
    << "  -a,--aim_file    Specify aim calibration file [aim_calib.yml]\n"
    << "  -i,--intrinsics  Specify camera intrinsics calibration file [calib.yml]\n"
    << std::endl;
}


int main(int argc, char* argv[] )
{

  devastar::PointSourceInf *pointSource = NULL;
  bool doCalibration = false;
  auto lastButton = devastar::BUTTON_NONE;
  std::string configFilename = "./config.yml";
  std::string aimCalibrationFilename = "./aim_calib.yml";
  std::string cameraCalibrationFilename = "./calib.yml";

  for ( int i=1; i < argc; ++i ) {
    std::string arg = argv[i];
    if ((arg == "-h") || (arg == "--help")) {
      usage(argv[0]);
      return 0;
    } else if ((arg == "-c") || (arg == "--calibrate")) {
      doCalibration = true;
    } else if ((arg == "-o") || (arg == "--config_file")) {
      // must have a filename
      if (i + 1 < argc) { 
        configFilename = argv[++i];
      } else { 
        std::cerr << "--configuration requires a filename." << std::endl;
        return 1;
      }  
    } else if ((arg == "-a") || (arg == "--aim_file")) {
      // must have a filename
      if (i + 1 < argc) { 
        aimCalibrationFilename = argv[++i];
      } else { 
        std::cerr << "--aim_file requires a filename." << std::endl;
        return 1;
      }  
    } else if ((arg == "-i") || (arg == "--intrinsics")) {
      // must have a filename
      if (i + 1 < argc) { 
        cameraCalibrationFilename = argv[++i];
      } else { 
        std::cerr << "--intrinsics requires a filename." << std::endl;
        return 1;
      }  
    } else {
      usage(argv[0]);
      return 0;
    }
  }

  // check configuration path
  fs::path configPath(configFilename);
  if( !fs::exists( configPath ) ) {
    std::cerr << "Configuration file '" << configFilename <<"' not found." << std::endl;
    return 1;
  }
  devastar::Configuration conf( configFilename );
  int frameWidth = conf.frameWidth;
  int frameHeight = conf.frameHeight;

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
  fs::path aimCalibrationPath(aimCalibrationFilename);

  devastar::AimCalibration ac;
  if( fs::exists( aimCalibrationPath ) ) {
    ac.readCalibrationFile( aimCalibrationPath );
  }
  devastar::AimCalibrator aimCalibrator(ac, 5, aimCalibrationFilename);

  std::cout << "Using: serial_device: " << conf.serialDevice << "\n";
  std::cout << "     ir_width: " << conf.irWidth << "\n";
  std::cout << "    ir_height: " << conf.irHeight << "\n";
  std::cout << " output_width: " << conf.outWidth << "\n";
  std::cout << "output_height: " << conf.outHeight << "\n";
  std::cout << " output_x_min: " << conf.outXMin << "\n";
  std::cout << " output_y_min: " << conf.outYMin << "\n";
  std::cout << "min_blob_size: " << conf.minBlobSize << "\n";
  std::cout << "max_blob_size: " << conf.maxBlobSize << "\n";
  std::cout << " ir_threshold: " << conf.irThreshold << "\n";
  std::cout << "  perspective: " << conf.usePerspectiveIntersection << "\n";

  std::cout << "        u_min: " << ac.uMin << "\n";
  std::cout << "        v_min: " << ac.vMin << "\n";
  std::cout << "        u_max: " << ac.uMax << "\n";
  std::cout << "        v_max: " << ac.vMax << "\n";
  std::cout << "    U/V width: " << ac.uWidth << "\n";
  std::cout << "   U/V height: " << ac.vHeight << "\n";

  bool use14BitData = conf.outWidth > 255.0f || conf.outHeight > 255.0f;
  std::cout << " use14BitData: " << use14BitData << "\n";

  // check calibration path
  fs::path calibPath( cameraCalibrationFilename );
  if( !fs::exists( calibPath ) ) {
    std::cerr << "No camera intrinsics calibration file found.  Please run ./bin/camera_calibrate." << std::endl;
    return -1;
  }
  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;
  cv::FileStorage fileStorage( calibPath, cv::FileStorage::READ);
  fileStorage["camera_matrix"] >> cameraMatrix;
  fileStorage["dist_coeffs"] >> distCoeffs;

  // setup GPIO 
  unsigned int offsets[5];
  int values[5];
  int gpioErr;
  g_chip = gpiod_chip_open("/dev/gpiochip0");
  if(!g_chip)
  {
    perror("gpiod_chip_open");
    return -1;
  }

  // setup button pins
  offsets[0] = 6;  // Trigger
  offsets[1] = 5;
  offsets[2] = 13;
  offsets[3] = 19; 
  offsets[4] = 26; 
  values[0] = -1;
  values[1] = -1;
  values[2] = -1;
  values[3] = -1;
  values[4] = -1;
  auto err = gpiod_chip_get_lines(g_chip, offsets, 5, &g_lines);
  if(err)
  {
    perror("gpiod_chip_get_lines");
    gpio_cleanup();
    return -2;
  }

  memset(&g_config, 0, sizeof(g_config));
  g_config.consumer = "devestar";
  g_config.request_type = GPIOD_LINE_REQUEST_DIRECTION_INPUT;
  g_config.flags = 0;

  // open lines for input
  err = gpiod_line_request_bulk(&g_lines, &g_config, values);
  if(err)
  {
    perror("gpiod_line_request_bulk");
    gpio_cleanup();
    return -3;
  }

  // setup cam device
  bool initRetval = false;
  if( conf.useDFRobot == true ) {
    std::cout << "Setup DFRobot" << std::endl;
    devastar::DFRobot *df = new devastar::DFRobot;
    initRetval = df->init( conf );
    std::cout << " DF Robot init: " << initRetval  << std::endl;
    pointSource = df;
  } else {
    std::cout << "Setup IR Camera with " << cameraCalibrationFilename  << std::endl;
    devastar::IRCam *ic = new devastar::IRCam;
    initRetval = ic->init( conf, cameraCalibrationFilename );
    std::cout << " IR Camera init: " << initRetval  << std::endl;
    pointSource = ic;
  }

  std::cout << "Pointsource is running()" <<  pointSource->isRunning() <<std::endl;
  if( !initRetval || !pointSource->isRunning() ) {
    std::cerr << "Camera failed to run" << std::endl;
    exit(-1);
  }



  // Setup World object points
  std::vector<cv::Point3f> worldPoints;
  worldPoints.push_back(cv::Point3f(0, 0, 0));
  worldPoints.push_back(cv::Point3f(conf.irWidth, 0, 0));
  worldPoints.push_back(cv::Point3f(0, conf.irHeight, 0));
  worldPoints.push_back(cv::Point3f(conf.irWidth, conf.irHeight, 0));

  // setup rectangle for PnP intersection 
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

  // setup target verticies for prespective intersection
  std::vector<cv::Point2f> targetVertices =  {
    cv::Point(0, 0), 
    cv::Point(conf.irWidth - 1, 0),
    cv::Point(0, conf.irHeight - 1),
    cv::Point(conf.irWidth - 1, conf.irHeight - 1)
  };

#ifdef SHOW_3D
  glutInit(&argc, argv);
  init3d(worldPoints);
#endif

  // Set up Serial Configuration
  struct termios serial;
  char buffer[BUFFER_SIZE];
  bool serialPortReady = true;
  int fd = open( conf.serialDevice.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if( fd < 0 ) {
    std::cerr << "Unable to open serial" << std::endl;
    perror( conf.serialDevice.c_str() );
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
  std::vector<cv::Point2f> undistortCenters;
  auto noArray = cv::noArray();
  undistortCenters.resize(4);

  // screen center point
  std::vector<cv::Point2f> srcPoints(1);
  srcPoints[0].x = float(frameWidth/2);
  srcPoints[0].y = float(frameHeight/2);

  // setup output
  unsigned char offscreen[] = { 0xFF, 0xFF, 0x00 };
  //unsigned char offscreen16[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0x00 };
  unsigned char xyb[3];  // 8 bits for x, y and buttons
  unsigned short sx, sy;
  unsigned char xxyyb[5]; // 14 bits for X and Y outputs. 7 bits for buttons
  unsigned char buttons = 0;

  // look at the cameras
  cv::Point2f pt;
  int ticks = 0;
  unsigned int lastSet = 0;
  while (true) {
    ++ticks;
#ifdef SHOW_3D
    glutMainLoopEvent();
#endif
    // Process buttons
    buttons = 0;
#ifdef SHOW_TIME
    auto loopTopTime = std::chrono::steady_clock::now();
    auto startTime = std::chrono::steady_clock::now();
#endif
    err = gpiod_line_get_value_bulk(&g_lines, values);
#ifdef SHOW_TIME
    auto endTime = std::chrono::steady_clock::now();
    std::cout << "ELAPSED TIME>> frame gpiod_line_get_value_bulk(): " << float(std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count()) / 1000.0f << "\n"; 
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
    if( !values[4] ) {
      buttons |= devastar::BUTTON_E;   // SWITCH
    }


    // get centers from source
    unsigned int currentSet = pointSource->getCenters( centers );
    if( currentSet != lastSet ) {
      lastSet = currentSet;
      // new set, we can process it
      bool goodCenters = true;
      // check for bad values.   
      for( const auto& iter: centers ) {
        // df robot res is 1024x768
        if( iter.y == 1023.0f  ) { 
          // too large for second value, must be bad
          goodCenters = false;
        }	
      }

      if ( goodCenters ) {
        bool solveRet = false;
        bool hit = false;
        // rvec- is the rotation vector
        // tvec- is the translation vector 
        float u,v;
        if( !conf.useDFRobot ) {
          // perspective transform 
          getPerspectiveIntersection( centers, targetVertices, srcPoints, u, v );
        }else  if(conf.usePerspectiveIntersection ) {
          // cameras have calibration files to undistort iamge
          cv::undistortPoints( centers, undistortCenters, cameraMatrix, distCoeffs, noArray, cameraMatrix );
          getPerspectiveIntersection( undistortCenters, targetVertices, srcPoints, u, v );
          solveRet = true;
        } else  {

          cv::Mat rvec, tvec;
          std::vector< cv::Mat > rvecs, tvecs;	
          solveRet = getPnPIntersection( worldPoints, centers, cameraMatrix, distCoeffs, P0, S1, S2, conf.irWidth, conf.irHeight, u, v);
#ifdef SHOW_3D
          update3d( tvec, R, u, v );
#endif
        }

        if( solveRet ) {
          hit = u > ac.uMin && u < ac.uMax && v > ac.vMin && v < ac.vMax;
          if( hit ) {
            // arduino has the min/max values  We just need the X/Y range calculated from the percentage
            auto outX = int(( (u-ac.uMin) / ac.uWidth) * conf.outWidth);
            auto outY = int(( (v-ac.vMin) / ac.vHeight) * conf.outHeight);
            if( !use14BitData ) {
              xyb[0] = (unsigned char)outX;
              xyb[1] = (unsigned char)outY;
              xyb[2] = buttons;
              if(serialPortReady ) {
                auto ret = write( fd, xyb, sizeof(xyb) );
              }
            } else {
              /* mouse mode is 1080p for now.
               *
               *
               sx = (unsigned short)outX;
               sy = (unsigned short)outY;
               memcpy( xxyyb, &sx, sizeof( unsigned short ) ); 
               memcpy( xxyyb + sizeof(unsigned short) , &sy, sizeof( unsigned short ) ); 
               */
              // bit7 determines starting char. This allows 14 bit numbers.  
              //  * first byte gets 128 + lower 7 bits of X
              //  * second byte gets bits 8 through 14 of x
              //  * third byte gets lower 7 bits of X
              //  * fourth byte gets bits 8 through 14 of x
              //  * fift byte gets buttons
              xxyyb[0] = 0x80 | (outX & 0x7F);
              xxyyb[1] = ( outX  >> 7 ) & 0x7F;
              xxyyb[2] = outY & 0x7F;
              xxyyb[3] = ( outY  >> 7 ) & 0x7F;
              xxyyb[4] = buttons;

#ifdef SHOW_CALC
              std::cout << " " <<  std::bitset<8>( xxyyb[0] )
                << " " <<  std::bitset<16>( xxyyb[1] )
                << " " <<  std::bitset<8>( xxyyb[2] )
                << " " <<  std::bitset<16>( xxyyb[3] )
                << " " <<  std::bitset<8>( xxyyb[4] )
                << std::endl;
#endif
              if(serialPortReady ) {
                auto ret = write( fd, xxyyb, sizeof(xxyyb) );
              }
            }
            //continue;  // head back up the loop
          } // if( hit ) 
        } // if( solveRet ) 



        if( doCalibration ) {
          // get state from aimcalibrator
          auto currentMode = aimCalibrator.getMode();
          auto currentShots = aimCalibrator.getCurrentSampleCount();
          auto maxShots = aimCalibrator.getMaxSamples();

          switch( currentMode ) {
            case devastar::AIM_CALIBRATE_UPPER_LEFT:
              std::cout << "\33[2K\rAim at upper left corner and pull trigger: " << currentShots << "/" << maxShots << "      " << std::flush;
              break;
            case devastar::AIM_CALIBRATE_LOWER_RIGHT:
              std::cout << "\33[2K\rAim at lower right corner and pull trigger: " << currentShots << "/" << maxShots << "      " << std::flush;
              break;
            case devastar::AIM_CALIBRATE_CALIBRATED:
              std::cout << "\33[2K\rCalibrated: B to reset calibration, C to Cancel, Start to Save          " << std::flush;
              break;
            case devastar::AIM_CALIBRATE_SAVED:
              std::cout << "\33[2K\rSaved: B to reset calibration, C or Start to exit      " << std::flush;
              break;
            default:
              break;
          }


          // A is trigger
          if( buttons & devastar::BUTTON_A ) {
            if( lastButton != devastar::BUTTON_A ) {
              lastButton = devastar::BUTTON_A;
              if( solveRet ) {
                // u and v were calcuatled, pass on to the calibrator
                aimCalibrator.appendSample( u, v );
              }
            } 
          } else if( buttons & devastar::BUTTON_B ) {
            if( lastButton != devastar::BUTTON_B ) {
              lastButton = devastar::BUTTON_B;
              // complety clear the values
              aimCalibrator.resetCalibration();
            } 
          } else if( buttons & devastar::BUTTON_C ) {
            if( lastButton != devastar::BUTTON_C ) {
              lastButton = devastar::BUTTON_C;
              if( currentMode == devastar::AIM_CALIBRATE_CALIBRATED ) {
                // go back to old calibration values
                aimCalibrator.cancelCalibration(); 
              } else {
                // exit out in all other cases
                std::cout << "\nExiting out\n";
                exit(0);
              }
            } 
          } else if( buttons & devastar::BUTTON_D ) {
            if( lastButton != devastar::BUTTON_D ) {
              lastButton = devastar::BUTTON_D;
              if( currentMode == devastar::AIM_CALIBRATE_CALIBRATED ) {
                // if calibrated, save it
                aimCalibrator.saveCalibration();
              } else if( currentMode == devastar::AIM_CALIBRATE_SAVED ) {
                // already saved, so exit
                std::cout << "\nExiting out\n";
                exit(0);
              } // ignore the rest
            } 
          } else if ( buttons == 0 ) {
            lastButton = devastar::BUTTON_NONE;
          }
        } // if( doCalibration ) 


        if( hit ) {
#ifdef SHOW_TIME
          auto loopBottomTime = std::chrono::steady_clock::now();
          std::cout << "ELAPSED TIME>> LOOP top to bottom" << float(std::chrono::duration_cast<std::chrono::microseconds>(loopTopTime - loopBottomTime).count()) / 1000.0f << "\n"; 
#endif
          continue;
        }

      } 
    }


    // send -1, -1 to arduino if offscreen
    if(serialPortReady ) {
      if( !use14BitData ) {
        offscreen[2] = buttons;
        auto ret = write( fd, offscreen, sizeof(offscreen) );
      } else {
        // Don't bother with offscreen for mouse
        //offscreen16[4] = buttons;
        //auto ret = write( fd, offscreen16, sizeof(offscreen16) );
      }
    }
#ifdef SHOW_TIME
    auto loopBottomTime = std::chrono::steady_clock::now();
    std::cout << "ELAPSED TIME>> LOOP top to bottom" << float(std::chrono::duration_cast<std::chrono::microseconds>(loopTopTime - loopBottomTime).count()) / 1000.0f << "\n"; 
#endif

  }// while(true)
  pointSource->stop();
}
