
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
    << "  -c,--calibrate   Run calibrate mode\n"
    << "  -o,--config_file Specify configuration file [config.yml]\n"
    << "  -a,--aim_file    Specify aim calibration file [aim_calib.yml]\n"
    << std::endl;
}


int main(int argc, char* argv[] )
{
  bool doCalibration = false;
  auto lastButton = devastar::BUTTON_NONE;
  std::string configFilename = "./config.yml";
  std::string aimCalibrationFilename = "./aim_calib.yml";

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

  std::cout << "        u_min: " << ac.uMin << "\n";
  std::cout << "        v_min: " << ac.vMin << "\n";
  std::cout << "        u_max: " << ac.uMax << "\n";
  std::cout << "        v_max: " << ac.vMax << "\n";
  std::cout << "    U/V width: " << ac.uWidth << "\n";
  std::cout << "   U/V height: " << ac.vHeight << "\n";

  bool use14BitData = conf.outWidth > 255.0f || conf.outHeight > 255.0f;
  std::cout << " use14BitData: " << use14BitData << "\n";

  // check calibration path
  fs::path calibPath("./calib.yml");
  if( !fs::exists( calibPath ) ) {
    std::cerr << "No camera intrinsics calibration file found.  Please run ./bin/camera_calibrate." << std::endl;
    return -1;
  }
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


  // Read in camera calibration calibration
  cv::FileStorage fileStorage(calibPath, cv::FileStorage::READ);
  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;
  fileStorage["camera_matrix"] >> cameraMatrix;
  fileStorage["dist_coeffs"] >> distCoeffs;


  // setup videocapture
  cv::VideoCapture inputVideo;
  inputVideo.open(0);
  const int frameWidth = 640.0f;
  const int frameHeight = 480.0f;
  inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, frameWidth);
  inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, frameHeight);
  std::cout << "CAP_PROP_BUFFERSIZE : " << inputVideo.set(cv::CAP_PROP_BUFFERSIZE,  1) << std::endl;


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
  cv::Mat frame;
  cv::Mat gray;
  cv::Mat thresh;
  cv::Mat displayCopy;

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

    // Process Video from camera
#ifdef SHOW_TIME
    startTime = std::chrono::steady_clock::now();
#endif
    inputVideo >> frame;
#ifdef SHOW_TIME
    endTime = std::chrono::steady_clock::now();
    std::cout << "ELAPSED TIME>> frame grab: " << float(std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count()) / 1000.0f << "\n"; 
#endif
    // cv to grey
#ifdef SHOW_TIME
    startTime = std::chrono::steady_clock::now();
#endif
    cv::cvtColor( frame, gray, cv::COLOR_BGR2GRAY); 
#ifdef SHOW_TIME
    endTime = std::chrono::steady_clock::now();
    std::cout << "ELAPSED TIME>> cvtColor(): " << float(std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count()) / 1000.0f << "\n"; 
#endif

    // threshold on brightness
#ifdef SHOW_TIME
    startTime = std::chrono::steady_clock::now();
#endif
    cv::threshold( gray, thresh, conf.irThreshold, 255, cv::THRESH_BINARY);
#ifdef SHOW_TIME
    endTime = std::chrono::steady_clock::now();
    std::cout << "ELAPSED TIME>> cv::threshold(): " << float(std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count()) / 1000.0f << "\n"; 
#endif
    std::vector< std::vector< cv::Point> > contours;
    // look for IR lights
#ifdef SHOW_TIME
    startTime = std::chrono::steady_clock::now();
#endif
    cv::findContours( thresh, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
#ifdef SHOW_TIME
    endTime = std::chrono::steady_clock::now();
    std::cout << "ELAPSED TIME>> cv::findContours(): " << float(std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count()) / 1000.0f << "\n"; 
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
        if( moments[i].m00 > conf.minBlobSize && moments[i].m00 < conf.maxBlobSize ) {

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

      bool hit = false;
      // only calculate hit if count is 4 (could potentially also check if "n >= 4", but 
      // use simplest case for now.
      if( centerCount == 4 ) {
#ifdef SHOW_TIME
        endTime = std::chrono::steady_clock::now();
        std::cout << "ELAPSED TIME>> get centers from moments: " << float(std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count()) / 1000.0f << "\n"; 
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
        std::cout << "ELAPSED TIME>> sort centers: " << float(std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count()) / 1000.0f << "\n"; 
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
        float u,v;
        if(conf.usePerspectiveIntersection ) {
          getPerspectiveIntersection( centers, targetVertices, srcPoints, u, v );
          solveRet = true;
        } else  {

          cv::Mat rvec, tvec;
          std::vector< cv::Mat > rvecs, tvecs;	
#ifdef SHOW_3D
          update3d( tvec, R, u, v );
#endif
        }

        if( solveRet ) {
          hit = u > ac.uMin && u < ac.uMax && v > ac.vMin && v < ac.vMax;
          if( hit ) {
#ifdef SHOW_IMAGE
            cv::Point2f pt;
            pt.x = (u/conf.irWidth) * float(frameWidth);
            pt.y = (v/conf.irHeight) * float(frameHeight); 
            cv::circle( displayCopy, pt, 5.0f, cv::Scalar(0,255,255), 3.0f);
            cv::imshow("points", displayCopy );
            cv::waitKey(1);
#endif
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

      } // if( centerCount == 4 ) 

    } // if (contours.size() >= 4) 


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
}
