#ifndef _DEVASTAR_COMMON_H_
#define _DEVASTAR_COMMON_H_

#include <vector>
#include <opencv2/opencv.hpp>

namespace devastar {

  enum ButtonCodes
  {
    BUTTON_NONE = 0x00,
    BUTTON_A = 0x01,
    BUTTON_B = 0x02,
    BUTTON_C = 0x04,
    BUTTON_D = 0x08,
    BUTTON_E = 0x10 
  };

  class PointSourceInf {
  public:
    virtual unsigned int getCenters( std::vector<cv::Point2f>& centers ) const = 0;
    virtual bool isRunning() = 0;
    virtual bool stop() = 0;
  };

}

#endif // _DEVASTAR_COMMON_H_
