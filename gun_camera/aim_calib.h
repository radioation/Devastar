#ifndef _AIM_CALIBRATION_H_
#define _AIM_CALIBRATION_H_

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>


namespace devastar {

  enum AimCalibrateMode {
    AIM_CALIBRATE_UPPER_LEFT,
    AIM_CALIBRATE_LOWER_RIGHT,
    AIM_CALIBRATE_START,
    AIM_CALIBRATE_CALIBRATED,
    AIM_CALIBRATE_SAVED
  };


  struct AimCalibration 
  {

    AimCalibration();
    AimCalibration(const float& u_min, 
                  const float& v_min,
                  const float& u_max,
                  const float& v_max);
                  
                 
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

  struct UVPt 
  {
    UVPt() : u(0.0f), v(0.0f) {};
    UVPt(const float&U, const float&V) : u(U), v(V) {};
    float u;
    float v;
  };

  class AimCalibrator 
  {
    public:
      AimCalibrator(devastar::AimCalibration & aimCalibration, 
                    unsigned int maxSamples,
                    const std::string& calibPath );
      ~AimCalibrator();

      AimCalibrateMode getMode() { return m_mode; };
      size_t getMaxSamples() { return m_maxSamples; };
      size_t getCurrentSampleCount() { return m_pointBuffer.size(); };

      size_t appendSample( const float&u, const float& v );

      // clear calibration to default values
      AimCalibrateMode resetCalibration();
      // cancle current values and restor the original settings
      void cancelCalibration();
      // save calibration to file. Will overwrite original settings with newly saved values
      void saveCalibration();



    private:
      AimCalibrator();
      AimCalibration &m_aimCalibration;
      AimCalibration m_aimCalibrationOrig;
      std::vector<UVPt> m_pointBuffer;
      AimCalibrateMode m_mode;
      unsigned int m_maxSamples;
      std::string m_calibrationPath;

      UVPt average( const std::vector<UVPt>& vec );
  };

}

#endif //  _AIM_CALIBRATION_H_

