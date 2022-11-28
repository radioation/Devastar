
#include "config.h"

#include <filesystem>
namespace fs = std::filesystem;

using namespace devastar;

Configuration::Configuration(const std::string& configFile) 
  : 
    serialDevice( DEFAULT_SERIAL_DEVICE ),
    irWidth(510.0f),
    irHeight(510.0f),
    outWidth(196.0f),
    outHeight(220),
    outXMin(73),
    outYMin(30),
    minBlobSize(6.0),
    maxBlobSize(250.0)
{
  fs::path configPath("./config.yml");

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
      fileStorage["output_y_min"] >> outXMin;
    }
    if(!fileStorage["output_height"].empty() ) {
      fileStorage["output_height"] >> outYMin;
    }

    if(!fileStorage["min_blob_size"].empty() ) {
      fileStorage["min_blob_size"] >> minBlobSize;
    }
    if(!fileStorage["max_blob_size"].empty() ) {
      fileStorage["max_blob_size"] >> maxBlobSize;
    }
  }
}
