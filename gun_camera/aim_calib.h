#ifndef _AIM_CALIBRATION_H_
#define _AIM_CALIBRATION_H_

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

namespace devastar {

  enum AimCalibrateMode {
    AIM_CALIBRATE_START,
    AIM_CALIBRATE_UPPER_LEFT,
    AIM_CALIBRATE_LOWER_RIGHT,
    AIM_CALIBRATE_RUN
  };


  struct AimCalibration 
  {

    AimCalibration(const float& irWidth, 
        const float& irHeight, 
        const float &outWidth,
        const float &outHeight,
        const float &outXMin,
        const float &outYMin
        );
    ~AimCalibration() {};

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
      AimCalibrator(devastar::AimCalibration & aimCalibration);
      ~AimCalibrator();

      size_t getCurrentSampleCount() { return m_pointBuffer.size(); };
      AimCalibrateMode getMode() { return m_mode; };
			size_t getMaxSamples() { return m_maxSamples; };

		private:
			AimCalibration &m_aimCalibration;
			std::vector<cv::Point2f> m_pointBuffer;
			AimCalibrateMode m_mode;
			unsigned int m_maxSamples;

	};

}

#endif //  _AIM_CALIBRATION_H_

