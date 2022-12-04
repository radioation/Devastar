
#include <filesystem>
#include <numeric>
namespace fs = std::filesystem;

#include "devastar_common.h"
#include "aim_calib.h"


using namespace devastar;

AimCalibration::AimCalibration() :
  uMin( 0.0f ),
  vMin( 0.0f ),
  uMax( 0.0f ),
  vMax( 0.0f ),
  uWidth( uMax - uMin),
  vHeight( vMax - vMin)
{
}


AimCalibration::AimCalibration(const float& u_min, 
                  const float& v_min,
                  const float& u_max,
                  const float& v_max ) :
  uMin( u_min ),
  vMin( v_min ),
  uMax( u_max ),
  vMax( v_max ),
  uWidth( uMax - uMin),
  vHeight( vMax - vMin)
{
}



bool AimCalibration::readCalibrationFile( const std::string& calibrationPath ) {
  if( fs::exists( calibrationPath ) ) {
    cv::FileStorage fileStorage(calibrationPath, cv::FileStorage::READ);

    if(!fileStorage["u_min"].empty() ) {
      fileStorage["u_min"] >> uMin;
    }
    if(!fileStorage["v_min"].empty() ) {
      fileStorage["v_min"] >> vMin;
    }
    if(!fileStorage["u_max"].empty() ) {
      fileStorage["u_max"] >> uMax;
    }
    if(!fileStorage["v_max"].empty() ) {
      fileStorage["v_max"] >> vMax;
    }
    uWidth = uMax - uMin;
    vHeight = vMax - vMin;

  } else {
    return false;
  }
  return true;
}

bool AimCalibration::writeCalibrationFile( const std::string& calibrationPath ){
  cv::FileStorage fileStorage(calibrationPath, cv::FileStorage::WRITE);

  fileStorage << "u_min" << uMin;
  fileStorage << "v_min" << vMin;
  fileStorage << "u_max" << uMax;
  fileStorage << "v_max" << vMax;

  return true;
};

AimCalibrator::AimCalibrator(AimCalibration &aimCalibration, 
                              unsigned int maxSamples, 
                              const std::string& calibrationPath) :
  m_aimCalibration(aimCalibration),
  m_aimCalibrationOrig(aimCalibration), 
  m_mode( AIM_CALIBRATE_UPPER_LEFT ),
  m_maxSamples(maxSamples),
  m_calibrationPath(calibrationPath) {
  }

AimCalibrator::~AimCalibrator() {
}


size_t AimCalibrator::appendSample(const float& u, const float &v) {
  m_uPointBuffer.push_back(u);
  m_vPointBuffer.push_back(v);

  if( m_uPointBuffer.size() >= m_maxSamples ) {
    if( m_mode == AIM_CALIBRATE_UPPER_LEFT ) {
      // find avg u and avg v, set to min
      m_aimCalibration.uMin = average( m_uPointBuffer );
      m_aimCalibration.vMin = average( m_vPointBuffer );
      m_uPointBuffer.clear();
      m_vPointBuffer.clear();

      m_mode = AIM_CALIBRATE_LOWER_RIGHT;
    }else if( m_mode == AIM_CALIBRATE_LOWER_RIGHT ) {
      // find avg u and avg y, set to max and width
      m_aimCalibration.uMax = average( m_uPointBuffer );
      m_aimCalibration.vMax = average( m_vPointBuffer );
      m_aimCalibration.uWidth = average( m_uPointBuffer ) - m_aimCalibration.uMin;
      m_aimCalibration.vHeight = average( m_vPointBuffer ) - m_aimCalibration.vMin;
      m_uPointBuffer.clear();
      m_vPointBuffer.clear();
      m_mode = AIM_CALIBRATE_CALIBRATED;
    }
  }
  return m_uPointBuffer.size();
}

AimCalibrateMode AimCalibrator::restartCalibration() {
  m_uPointBuffer.clear();
  m_vPointBuffer.clear();
  m_mode = AIM_CALIBRATE_UPPER_LEFT;
  return m_mode;
}


void AimCalibrator::cancelCalibration() {
  restartCalibration();
  m_aimCalibration = m_aimCalibrationOrig;
}

void AimCalibrator::saveCalibration() {
  m_aimCalibration.writeCalibrationFile( m_calibrationPath );
  m_aimCalibrationOrig =  m_aimCalibration;
  restartCalibration();
}

float AimCalibrator::average( const std::vector<float>& vec ) {
  if( vec.empty() ) {
    return 0.0f;
  }
  return std::reduce( vec.begin(), vec.end() ) / (float)vec.size();
}

