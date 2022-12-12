
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
  std::string testOutfile2 = "ac_calib.yml";
  devastar::AimCalibration ac;
  devastar::AimCalibrator aimCalibrator(ac, maxSamples, testOutfile2);

  // default mode is AIM_CALIBRATE_UPPER_LEFT
  std::cout  << "Start State " << std::endl;
	CHECK_EQUAL_INT( aimCalibrator.getMode(), devastar::AIM_CALIBRATE_UPPER_LEFT, "Check Calibrator Mode" );
  // max should be 5
	CHECK_EQUAL_INT( aimCalibrator.getMaxSamples(), maxSamples, "Check Max Samples");

  // current should be zero the first time
	CHECK_EQUAL_INT( aimCalibrator.getCurrentSampleCount(), 0, "Check starting sample count");

  // add more than max
  float u = -100.0f;
  float v = -100.0f;
  size_t count = aimCalibrator.appendSample(u,v);
	CHECK_EQUAL_INT( count, 1, "Check return val after append");
	CHECK_EQUAL_INT( aimCalibrator.getCurrentSampleCount(), 1, "Check starting sample count after append");
	CHECK_EQUAL_INT( aimCalibrator.getMode(), devastar::AIM_CALIBRATE_UPPER_LEFT, "Check Calibrator Mode is still UPPER LEFT" );
  u = -110.0f;
  v = -110.0f;
  count = aimCalibrator.appendSample(u,v);
	CHECK_EQUAL_INT( count, 2, "Check return val after append");
	CHECK_EQUAL_INT( aimCalibrator.getCurrentSampleCount(), 2, "Check starting sample count after append");
	CHECK_EQUAL_INT( aimCalibrator.getMode(), devastar::AIM_CALIBRATE_UPPER_LEFT, "Check Calibrator Mode is still UPPER LEFT" );
  u = -105.0f;
  v = -105.0f;
  count = aimCalibrator.appendSample(u,v);
	CHECK_EQUAL_INT( count, 3, "Check return val after append");
	CHECK_EQUAL_INT( aimCalibrator.getCurrentSampleCount(), 3, "Check starting sample count after append");
	CHECK_EQUAL_INT( aimCalibrator.getMode(), devastar::AIM_CALIBRATE_UPPER_LEFT, "Check Calibrator Mode is still UPPER LEFT" );
  u = -102.5f;
  v = -102.5f;
  count = aimCalibrator.appendSample(u,v);
	CHECK_EQUAL_INT( count, 4, "Check return val after append");
	CHECK_EQUAL_INT( aimCalibrator.getCurrentSampleCount(), 4, "Check starting sample count after append");
	CHECK_EQUAL_INT( aimCalibrator.getMode(), devastar::AIM_CALIBRATE_UPPER_LEFT, "Check Calibrator Mode is still UPPER LEFT" );
  u = -107.5f;
  v = -107.5f;
  count = aimCalibrator.appendSample(u,v);  // will change state
	CHECK_EQUAL_REAL( ac.uMin, -105.0f, "ac.uMin", 0.00001);
	CHECK_EQUAL_REAL( ac.vMin, -105.0f, "ac.vMin", 0.00001);
	CHECK_EQUAL_INT( aimCalibrator.getCurrentSampleCount(), 0, "Check starting sample count after append");


  std::cout  << "New State " << std::endl;
	CHECK_EQUAL_INT( aimCalibrator.getMode(), devastar::AIM_CALIBRATE_LOWER_RIGHT, "Check Calibrator Mode is now LOWER RIGHT" );
  u = 200.0f;
  v = 150.0f;
  count = aimCalibrator.appendSample(u,v);
	CHECK_EQUAL_INT( count, 1, "Check return val after append");
	CHECK_EQUAL_INT( aimCalibrator.getCurrentSampleCount(), 1, "Check starting sample count after append");
	CHECK_EQUAL_INT( aimCalibrator.getMode(), devastar::AIM_CALIBRATE_LOWER_RIGHT, "Check Calibrator Mode is still LOWER RIGHT " );
  u = 210.0f;
  v = 160.0f;
  count = aimCalibrator.appendSample(u,v);
	CHECK_EQUAL_INT( count, 2, "Check return val after append");
	CHECK_EQUAL_INT( aimCalibrator.getCurrentSampleCount(), 2, "Check starting sample count after append");
	CHECK_EQUAL_INT( aimCalibrator.getMode(), devastar::AIM_CALIBRATE_LOWER_RIGHT, "Check Calibrator Mode is still LOWER RIGHT " );

  u = 190.0f;
  v = 140.0f;
  count = aimCalibrator.appendSample(u,v);
	CHECK_EQUAL_INT( count, 3, "Check return val after append");
	CHECK_EQUAL_INT( aimCalibrator.getCurrentSampleCount(), 3, "Check starting sample count after append");
	CHECK_EQUAL_INT( aimCalibrator.getMode(), devastar::AIM_CALIBRATE_LOWER_RIGHT, "Check Calibrator Mode is still LOWER RIGHT " );
  u = 205.0f;
  v = 155.0f;
  count = aimCalibrator.appendSample(u,v);
	CHECK_EQUAL_INT( count, 4, "Check return val after append");
	CHECK_EQUAL_INT( aimCalibrator.getCurrentSampleCount(), 4, "Check starting sample count after append");
	CHECK_EQUAL_INT( aimCalibrator.getMode(), devastar::AIM_CALIBRATE_LOWER_RIGHT, "Check Calibrator Mode is still LOWER RIGHT " );
  u = 195.0f;
  v = 145.0f;
  count = aimCalibrator.appendSample(u,v);
	CHECK_EQUAL_INT( count, 0, "Check return val after append");
	CHECK_EQUAL_INT( aimCalibrator.getCurrentSampleCount(), 0, "Check starting sample count after append");
	CHECK_EQUAL_INT( aimCalibrator.getMode(), devastar::AIM_CALIBRATE_CALIBRATED, "Check Calibrator Mode is now CALIBRATED " );
	CHECK_EQUAL_REAL( ac.uMax, 200.0f, "ac.uMax", 0.00001);
	CHECK_EQUAL_REAL( ac.vMax, 150.0f, "ac.vMax", 0.00001);
	CHECK_EQUAL_REAL( ac.uWidth, 305.0f, "ac.uWidth", 0.00001);
	CHECK_EQUAL_REAL( ac.vHeight, 255.0f, "ac.vHeight", 0.00001);

  // reset willl go back to the original calib which was default 0 everywhere
  std::cout << "Reset" << std::endl; 
  aimCalibrator.cancelCalibration();
	CHECK_EQUAL_REAL( ac.uMin, 0.0f, "cancel test 1 ac.uMin", 0.00001);
	CHECK_EQUAL_REAL( ac.vMin, 0.0f, "cancel test 1 ac.vMin", 0.00001);
	CHECK_EQUAL_REAL( ac.uMax, 0.0f, "cancel test 1 ac.uMax", 0.00001);
	CHECK_EQUAL_REAL( ac.vMax, 0.0f, "cancel test 1 ac.vMax", 0.00001);
	CHECK_EQUAL_REAL( ac.uWidth, 0.0f, "cancel test 1 ac.uWidth", 0.00001);
	CHECK_EQUAL_REAL( ac.vHeight, 0.0f, "cancel test 1 ac.vHeight", 0.00001);
	CHECK_EQUAL_INT( aimCalibrator.getMode(), devastar::AIM_CALIBRATE_UPPER_LEFT, "Check Calibrator Mode is now  UPPER LEFT after cancel" );


  // add left samples
  u = -10.0f;
  v = -10.0f;
  for( int i=0; i < maxSamples; ++i ) {
    aimCalibrator.appendSample(u,v);
  }
  u = 110.0f;
  v = 110.0f;
  for( int i=0; i < maxSamples; ++i ) {
    aimCalibrator.appendSample(u,v);
  }
	CHECK_EQUAL_REAL( ac.uMin, -10.0f, "ac.uMin", 0.00001);
	CHECK_EQUAL_REAL( ac.vMin, -10.0f, "ac.vMin", 0.00001);
	CHECK_EQUAL_REAL( ac.uMax, 110.0f, "ac.uMax", 0.00001);
	CHECK_EQUAL_REAL( ac.vMax, 110.0f, "ac.vMax", 0.00001);
	CHECK_EQUAL_REAL( ac.uWidth, 120.0f, "ac.uWidth", 0.00001);
	CHECK_EQUAL_REAL( ac.vHeight, 120.0f, "ac.vHeight", 0.00001);
	CHECK_EQUAL_INT( aimCalibrator.getMode(), devastar::AIM_CALIBRATE_CALIBRATED, "Check Calibrator Mode is now CALIBRATED " );

  std::cout << "Save calibration" << std::endl;
  // should write to file and set -10,110,-10,110 as the *current* calibration
  aimCalibrator.saveCalibration();
  // Canceling at this point will keep -10,110,-10,111 as it was saved in the previous test
  aimCalibrator.cancelCalibration();
	CHECK_EQUAL_REAL( ac.uMin, -10.0f, "cancel test 2 ac.uMin", 0.00001);
	CHECK_EQUAL_REAL( ac.vMin, -10.0f, "cancel test 2 ac.vMin", 0.00001);
	CHECK_EQUAL_REAL( ac.uMax, 110.0f, "cancel test 2 ac.uMax", 0.00001);
	CHECK_EQUAL_REAL( ac.vMax, 110.0f, "cancel test 2 ac.vMax", 0.00001);
	CHECK_EQUAL_REAL( ac.uWidth, 120.0f, "cancel test 2 ac.uWidth", 0.00001);
	CHECK_EQUAL_REAL( ac.vHeight, 120.0f, "cancel test 2 ac.vHeight", 0.00001);
	CHECK_EQUAL_INT( aimCalibrator.getMode(), devastar::AIM_CALIBRATE_UPPER_LEFT, "Check Calibrator Mode is now UPPER LEFT " );

  std::cout << "Check saved file" << std::endl;
  // Check file by reading another calibration and compare 
  devastar::AimCalibration ac2;
  ac2.readCalibrationFile( testOutfile2 );
	CHECK_EQUAL_REAL( ac2.uMin, -10.0f, "read in ac2.uMin", 0.00001);
	CHECK_EQUAL_REAL( ac2.vMin, -10.0f, "read in ac2.vMin", 0.00001);
	CHECK_EQUAL_REAL( ac2.uMax, 110.0f, "read in ac2.uMax", 0.00001);
	CHECK_EQUAL_REAL( ac2.vMax, 110.0f, "read in ac2.vMax", 0.00001);
	CHECK_EQUAL_REAL( ac2.uWidth, 120.0f, "read in ac2.uWidth", 0.00001);
	CHECK_EQUAL_REAL( ac2.vHeight, 120.0f, "read in ac2.vHeight", 0.00001);
	CHECK_EQUAL_INT( aimCalibrator.getMode(), devastar::AIM_CALIBRATE_UPPER_LEFT, "Check Calibrator Mode is now UPPER LEFT " );

}
