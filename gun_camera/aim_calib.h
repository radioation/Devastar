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
    AIM_CALIBRATE_CALIBRATED,
    AIM_CALIBRATE_SAVED
  };


  struct AimCalibration 
  {

    AimCalibration() {};
    AimCalibration(const float& ir_width, 
        const float& ir_height, 
        const float &out_x_min,
        const float &out_y_min,
        const float &out_x_max,
        const float &out_y_max
        );
    ~AimCalibration() {};

    // IR LED model width/height.
    float irWidth; 
    float irHeight; 

    // X/Y values to receiver.
    float outXMin;
    float outYMin;
    float outXMax;
    float outYMax;
    float outWidth;
    float outHeight;

    // U/V bounds for screen in camera space.
    float uMin;
    float vMin;
    float uMax;
    float vMax;
    float uWidth;
    float vHeight;

    bool readConfig( const std::string& configPath );
    bool writeConfig( const std::string& configPath );
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
      std::string m_configPath;

      float average( const std::vector<float>& vec );
  };

}

#endif //  _AIM_CALIBRATION_H_

