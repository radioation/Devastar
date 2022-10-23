
#include <filesystem>
#include <numeric>
namespace fs = std::filesystem;

#include "devastar_common.h"
#include "aim_calib.h"


using namespace devastar;

AimCalibration::AimCalibration(const float& ir_width,
    const float& ir_height,
    const float &out_x_min,
    const float &out_y_min,
    const float &out_x_max,
    const float &out_y_max
    ) :
  irWidth(  ir_width ),
  irHeight( ir_height ),
  outXMin( out_x_min ),
  outYMin( out_y_min ),
  outXMax( out_x_max ),
  outYMax( out_y_max ),
  outWidth( outXMax-outXMin ),
  outHeight( outYMax-outYMin ),
  uMin( 0.0f ),
  vMin( 0.0f ),
  uMax( irWidth ),
  vMax( irHeight )
{
}



bool AimCalibration::readConfig( const std::string& configPath ) {
  if( fs::exists( configPath ) ) {
    cv::FileStorage fileStorage(configPath, cv::FileStorage::READ);
    if(!fileStorage["ir_width"].empty() ) {
      fileStorage["ir_width"] >> irWidth;
    }
    if(!fileStorage["ir_height"].empty() ) {
      fileStorage["ir_height"] >> irHeight;
    }
    if(!fileStorage["output_x_min"].empty() ) {
      fileStorage["output_x_min"] >> outXMin;
    }
    if(!fileStorage["output_y_min"].empty() ) {
      fileStorage["output_y_min"] >> outYMin;
    }
    if(!fileStorage["output_x_max"].empty() ) {
      fileStorage["output_x_max"] >> outXMax;
    }
    if(!fileStorage["output_y_max"].empty() ) {
      fileStorage["output_y_max"] >> outYMax;
    }
    outWidth = outXMax - outXMin;
    outHeight = outYMax - outYMin;

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

bool AimCalibration::writeConfig( const std::string& configPath ){
  cv::FileStorage fileStorage(configPath, cv::FileStorage::APPEND);
  fileStorage << "ir_width" << irWidth;
  fileStorage << "ir_height" << irHeight;

  fileStorage << "output_x_min" << outXMin;
  fileStorage << "output_y_min" << outYMin;
  fileStorage << "output_x_max" << outXMax;
  fileStorage << "output_y_max" << outYMax;

  fileStorage << "u_min" << uMin;
  fileStorage << "v_min" << vMin;
  fileStorage << "u_max" << uMax;
  fileStorage << "v_max" << vMax;

  return true;
};

AimCalibrator::AimCalibrator(AimCalibration &aimCalibration, 
                              unsigned int maxSamples, 
                              const std::string& configPath) :
  m_aimCalibration(aimCalibration),
  m_aimCalibrationOrig(aimCalibration), 
  m_mode( AIM_CALIBRATE_UPPER_LEFT ),
  m_maxSamples(maxSamples),
  m_configPath(configPath) {
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
  m_aimCalibration.writeConfig( m_configPath );
  m_aimCalibrationOrig =  m_aimCalibration;
  restartCalibration();
}

float AimCalibrator::average( const std::vector<float>& vec ) {
  if( vec.empty() ) {
    return 0.0f;
  }
  return std::reduce( vec.begin(), vec.end() ) / (float)vec.size();
}

