#ifndef _DEV_CONFIGURATION_H_
#define _DEV_CONFIGURATION_H_

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

#define DEFAULT_SERIAL_DEVICE "/dev/ttyACM0"


namespace devastar {

  struct Configuration 
  {

    Configuration() {};
    Configuration(const std::string& );
    ~Configuration() {};

    // serical device
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

  };
}
#endif //  _DEV_CONFIGURATION_H_

