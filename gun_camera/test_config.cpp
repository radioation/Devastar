
#include <fstream>

#include "test.h"
#include "config.h"
#include "devastar_common.h"



int main(int argc, char* argv[] )
{
   
  std::string serial_device =  "/dev/fakecomm0";
  float ir_width = 555.0f;
  float ir_height = 275.0f;
  float output_width = 1900.0f;
  float output_height = 1600.0f;
  float output_x_min = 1.0f;
  float output_y_min = 1.5f;
  double min_blob_size = 2.75;
  double max_blob_size = 125.0;
  float ir_threshold = 112.0;
  bool use_perspective_intersection = true;
  std::string i2c_device =  "/dev/fakei2c";

  std::string configfile = "testconfig.yml";
  std::ofstream testconfig(configfile);

  testconfig << "%YAML:1.0\n";
  testconfig << "---\n";
  testconfig << "serial_device: " << serial_device << std::endl;
  testconfig << "ir_width: " << ir_width << std::endl;
  testconfig << "ir_height: " << ir_height << std::endl;
  testconfig << "output_width: " << output_width  << std::endl;
  testconfig << "output_height: " << output_height << std::endl;
  testconfig << "output_x_min: " << output_x_min << std::endl;
  testconfig << "output_y_min: " << output_y_min << std::endl;
  testconfig << "min_blob_size: " << min_blob_size << std::endl;
  testconfig << "max_blob_size: " << max_blob_size << std::endl;
  testconfig << "ir_threshold: " << ir_threshold << std::endl;
  testconfig << "use_perspective_intersection:" << use_perspective_intersection << std::endl;
  testconfig << "i2c_device: " << i2c_device << std::endl;
  testconfig.close();

  devastar::Configuration config( configfile );

	CHECK_EQUAL( config.serialDevice, serial_device, "Serial Device Name");
	CHECK_EQUAL_REAL( config.irWidth, ir_width, "ir_width", 0.00001);
	CHECK_EQUAL_REAL( config.irHeight, ir_height, "ir_height", 0.00001);
	CHECK_EQUAL_REAL( config.outWidth, output_width, "output_width", 0.00001);
	CHECK_EQUAL_REAL( config.outHeight,output_height, "output_height", 0.00001);
	CHECK_EQUAL_REAL( config.outXMin, output_x_min, "output_x_min", 0.00001);
	CHECK_EQUAL_REAL( config.outYMin, output_y_min, "output_y_min", 0.00001);
	CHECK_EQUAL_REAL( config.minBlobSize, min_blob_size, "min_blob_size", 0.00001);
	CHECK_EQUAL_REAL( config.maxBlobSize, max_blob_size, "max_blob_size", 0.00001);
	CHECK_EQUAL_REAL( config.irThreshold, ir_threshold, "ir_threshold", 0.00001);
	CHECK_EQUAL( config.usePerspectiveIntersection, use_perspective_intersection, "Use Perspective Intersection");
	CHECK_EQUAL( config.i2cDevice, i2c_device, "I2C Device Name");
}
