#include "devastar_common.h"
#include "aim_calib.h"

#include <filesystem>
namespace fs = std::filesystem;

using namespace devastar;

AimCalibration::AimCalibration(const float& irWidth,
    const float& irHeight,
    const float &outWidth,
    const float &outHeight,
    const float &outXMin,
    const float &outYMin
    ) :
  irWidth(  irWidth ),
  irHeight( irHeight ),
  outWidth( outWidth  ),
  outHeight( outHeight ),
  outXMin( outXMin ),
  outYMin( outYMin )
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
    if(!fileStorage["output_width"].empty() ) {
      fileStorage["output_width"] >> outWidth;
    }
    if(!fileStorage["output_wheight"].empty() ) {
      fileStorage["output_height"] >> outHeight;
    }
    if(!fileStorage["output_x_min"].empty() ) {
      fileStorage["output_x_min"] >> outXMin;
    }
    if(!fileStorage["output_y_min"].empty() ) {
      fileStorage["output_y_min"] >> outYMin;
    }

  } else {
    return false;
  }
  return true;
}
bool AimCalibration::writeConfig( const std::string& configPath ){
  return true;
};

AimCalibrator::AimCalibrator(AimCalibration &aimCalibration) :
  m_aimCalibration(aimCalibration),
  m_mode( AIM_CALIBRATE_UPPER_LEFT ), m_maxSamples(10) {
  }

AimCalibrator::~AimCalibrator() {
}





