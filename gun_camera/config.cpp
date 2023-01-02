
#include "config.h"

#include <filesystem>
namespace fs = std::filesystem;

using namespace devastar;

Configuration::Configuration(const std::string& configFile) 
  : 
    serialDevice( DEFAULT_SERIAL_DEVICE ),
    irWidth(600.0f),
    irHeight(600.0f),
    outWidth(320.0f),
    outHeight(220.0f),
    outXMin(10.0f),
    outYMin(5.0f),
    minBlobSize(3.0),
    maxBlobSize(275.0),
    irThreshold(127.0),
    usePerspectiveIntersection(false),
    i2cDevice( DEFAULT_I2C_DEVICE )
{
  fs::path configPath(configFile);

  if( fs::exists( configPath ) ) {
    cv::FileStorage fileStorage(configPath, cv::FileStorage::READ);

    if(!fileStorage["serial_device"].empty() ) {
      fileStorage["serial_device"] >> serialDevice;
    }
    if(!fileStorage["ir_width"].empty() ) {
      fileStorage["ir_width"] >> irWidth;
    }
    if(!fileStorage["ir_width"].empty() ) {
      fileStorage["ir_height"] >> irHeight;
    }

    if(!fileStorage["output_width"].empty() ) {
      fileStorage["output_width"] >> outWidth;
    }
    if(!fileStorage["output_height"].empty() ) {
      fileStorage["output_height"] >> outHeight;
    }

    if(!fileStorage["output_x_min"].empty() ) {
      fileStorage["output_x_min"] >> outXMin;
    }
    if(!fileStorage["output_y_min"].empty() ) {
      fileStorage["output_y_min"] >> outYMin;
    }

    if(!fileStorage["min_blob_size"].empty() ) {
      fileStorage["min_blob_size"] >> minBlobSize;
    }
    if(!fileStorage["max_blob_size"].empty() ) {
      fileStorage["max_blob_size"] >> maxBlobSize;
    }

    if(!fileStorage["ir_threshold"].empty() ) {
      fileStorage["ir_threshold"] >> irThreshold;
    }

    if(!fileStorage["use_perspective_intersection"].empty() ) {
      fileStorage["use_perspective_intersection"] >> usePerspectiveIntersection;
    }

    if(!fileStorage["i2c_device"].empty() ) {
      fileStorage["i2c_device"] >> i2cDevice;
    }
  }
}
