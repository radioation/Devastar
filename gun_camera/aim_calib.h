#ifndef _AIM_CALIBRATION_H_
#define _AIM_CALIBRATION_H_

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

#include "config.h"

namespace devastar {

  enum AimCalibrateMode {
    AIM_CALIBRATE_START,
    AIM_CALIBRATE_UPPER_LEFT,
    AIM_CALIBRATE_LOWER_RIGHT,
    AIM_CALIBRATE_CALIBRATED,
    AIM_CALIBRATE_SAVED
  };


  struct AimCalibration 
  {

    AimCalibration(const Configuration& config );
    ~AimCalibration() {};

    // U/V bounds for screen in camera space.
    float uMin;
    float vMin;
    float uMax;
    float vMax;
    float uWidth;
    float vHeight;

    bool readCalibrationFile( const std::string& calibPath );
    bool writeCalibrationFile( const std::string& calibPath );
  };

  class AimCalibrator 
  {
    public:
      AimCalibrator(devastar::AimCalibration & aimCalibration, 
                    unsigned int maxSamples,
                    const std::string& configPath );
      ~AimCalibrator();

      size_t getCurrentSampleCount() { return m_uPointBuffer.size(); };
      AimCalibrateMode getMode() { return m_mode; };
      size_t getMaxSamples() { return m_maxSamples; };

      size_t appendSample( const float&u, const float& v );

      AimCalibrateMode restartCalibration();
      void cancelCalibration();
      void saveCalibration();



    private:
      AimCalibrator();
      AimCalibration &m_aimCalibration;
      AimCalibration m_aimCalibrationOrig;
      std::vector<float> m_uPointBuffer;
      std::vector<float> m_vPointBuffer;
      AimCalibrateMode m_mode;
      unsigned int m_maxSamples;
      std::string m_calibrationPath;

      float average( const std::vector<float>& vec );
  };

}

#endif //  _AIM_CALIBRATION_H_

