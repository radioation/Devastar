#ifndef _AIM_CALIBRATION_H_
#define _AIM_CALIBRATION_H_

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

namespace devastar {

  enum AimCalibrateMode {
    AIM_CALIBRATE_UNFILTERED,
    AIM_CALIBRATE_MEDIAN,
    AIM_CALIBRATE_AVERAGE
  };


  struct AimCalibration 
  {
    AimCalibration();
    ~AimCalibration();
    float irWidth; 
    float irHeight; 
    float outWidth;
    float outHeight;
    float outXMin;
    float outYMin;
    bool readConfig( const std::string& configPath );
    bool writeConfig( const std::string& configPath );
  };

  class AimCalibrator 
  {
    public:
      AimCalibrator();
      ~AimCalibrator();

    private:
      std::vector<cv::Point2f> m_pointBuffer;
      AimCalibrateMode m_mode;

  };

}

#endif //  _AIM_CALIBRATION_H_

