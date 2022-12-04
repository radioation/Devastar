
#include <fstream>

#include "test.h"
#include "aim_calib.h"
#include "devastar_common.h"



int main(int argc, char* argv[] )
{

  // setup calibratION struct
  devastar::AimCalibration ac0;
	CHECK_EQUAL_REAL( ac0.uMin, 0.0, "default ctor uMin", 0.00001);
	CHECK_EQUAL_REAL( ac0.vMin, 0.0, "default ctor vMin", 0.00001);
	CHECK_EQUAL_REAL( ac0.uMax, 0.0, "default ctor uMax", 0.00001);
	CHECK_EQUAL_REAL( ac0.vMax, 0.0, "default ctor vMax", 0.00001);
	CHECK_EQUAL_REAL( ac0.uWidth, 0.0, "default ctor uWidth", 0.00001);
	CHECK_EQUAL_REAL( ac0.vHeight, 0.0, "default ctor vHeight", 0.00001);

  float u_min = 1.0f;
  float v_min = 2.0f;
  float u_max = 10.0f;
  float v_max = 20.0f;

  devastar::AimCalibration ac1( u_min, v_min, u_max, v_max );
   
	CHECK_EQUAL_REAL( ac1.uMin, u_min, "param ctor uMin", 0.00001);
	CHECK_EQUAL_REAL( ac1.vMin, v_min, "param ctor vMin", 0.00001);
	CHECK_EQUAL_REAL( ac1.uMax, u_max, "param ctor uMax", 0.00001);
	CHECK_EQUAL_REAL( ac1.vMax, v_max, "param ctor vMax", 0.00001);
	CHECK_EQUAL_REAL( ac1.uWidth, (u_max - u_min), "param ctor uWidth", 0.00001);
	CHECK_EQUAL_REAL( ac1.vHeight, (v_max - v_min), "param ctor vHeight", 0.00001);


  std::string testOutfile1 = "ac1_calib.yml";
  // write and read file
  ac1.writeCalibrationFile( testOutfile1 );
  ac0.readCalibrationFile( testOutfile1 );
	CHECK_EQUAL_REAL( ac0.uMin, ac1.uMin, "write read uMin", 0.00001);
	CHECK_EQUAL_REAL( ac0.vMin, ac1.vMin, "write read vMin", 0.00001);
	CHECK_EQUAL_REAL( ac0.uMax, ac1.uMax, "write read uMax", 0.00001);
	CHECK_EQUAL_REAL( ac0.vMax, ac1.vMax, "write read vMax", 0.00001);
	CHECK_EQUAL_REAL( ac0.uWidth, ac1.uWidth, "write read uWidth", 0.00001);
	CHECK_EQUAL_REAL( ac0.vHeight, ac1.vHeight, "write read vHeight", 0.00001);
  ac1.uMin = 0.0f;
  ac1.vMin = 0.0f;
  ac1.uMax = 500.0f;
  ac1.vMax = 272.0f;
  ac1.uWidth = ac1.uMax - ac1.uMin;
  ac1.vHeight = ac1.vMax - ac1.vMin;

  ac1.writeCalibrationFile( testOutfile1 );
  ac0.readCalibrationFile( testOutfile1 );
	CHECK_EQUAL_REAL( ac0.uMin, ac1.uMin, "write read 2 uMin", 0.00001);
	CHECK_EQUAL_REAL( ac0.vMin, ac1.vMin, "write read 2 vMin", 0.00001);
	CHECK_EQUAL_REAL( ac0.uMax, ac1.uMax, "write read 2 uMax", 0.00001);
	CHECK_EQUAL_REAL( ac0.vMax, ac1.vMax, "write read 2 vMax", 0.00001);
	CHECK_EQUAL_REAL( ac0.uWidth, ac1.uWidth, "write read 2 uWidth", 0.00001);
	CHECK_EQUAL_REAL( ac0.vHeight, ac1.vHeight, "write read 2 vHeight", 0.00001);


  // calibratOR

  // setup known config
  std::string serial_device =  "/dev/fakecomm0";
  float ir_width = 555.0f;
  float ir_height = 275.0f;
  float output_width = 1900.0f;
  float output_height = 1600.0f;
  float output_x_min = 1.0f;
  float output_y_min = 1.5f;
  double min_blob_size = 2.75;
  double max_blob_size = 125.0;

  std::string configfile = "testconfig4calib.yml";
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
  testconfig.close();

 
  int maxSamples = 5;
  devastar::AimCalibration ac;
  devastar::AimCalibrator aimCalibrator(ac, maxSamples, configfile );

  // default mode is AIM_CALIBRATE_UPPER_LEFT
	CHECK_EQUAL_INT( aimCalibrator.getMode(), devastar::AIM_CALIBRATE_UPPER_LEFT, "Check Calibrator Mode" );
  // max should be 5
	CHECK_EQUAL_INT( aimCalibrator.getMaxSamples(), maxSamples, "Check Max Samples");

  // current should be zero the first time
	CHECK_EQUAL_INT( aimCalibrator.getCurrentSampleCount(), 0, "Check starging sample count");

  // add more than max
  float u = 100.0f;
  float v = 100.0f;
  for( int i=0; i > maxSamples +1; ++i ) {
    aimCalibrator.appendSampleAndCalculate(u,v);
  }

/*
  aimCalibrator.startCalibration();
  aimCalibrator.cancelCalibration();
  aimCalibrator.saveCalibration();
  */
}
