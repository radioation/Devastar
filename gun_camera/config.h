#ifndef _DEV_CONFIGURATION_H_
#define _DEV_CONFIGURATION_H_

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

#define DEFAULT_SERIAL_DEVICE "/dev/ttyACM0"
#define DEFAULT_I2C_DEVICE "/dev/i2c-1"


namespace devastar {

  struct Configuration 
  {

    Configuration() {};
    Configuration(const std::string& );
    ~Configuration() {};

    // serial device
    std::string serialDevice;
    // IR LED model width/height.
    float irWidth; 
    float irHeight; 

    // X/Y values to receiver.
    float outWidth;
    float outHeight;
    float outXMin;
    float outYMin;

    // ir blog sizes
    double minBlobSize;
    double maxBlobSize;

    // threshold
    float irThreshold;

    bool usePerspectiveIntersection;

    // I2C  device
    std::string i2cDevice;
    bool useDFRobot;

    // camera 
    int frameWidth;
    int frameHeight;
    bool colorCamera;

  };
}
#endif //  _DEV_CONFIGURATION_H_

