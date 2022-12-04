
#include <fstream>

#include "test.h"
#include "aim_calib.h"
#include "devastar_common.h"



int main(int argc, char* argv[] )
{

  // setup config object.
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
  

  /*
  devastar::AimCalibration ac( config );




  devastar::AimCalibrator aimCalibrator(ac, 5, configPath.string() );

  auto currentMode = aimCalibrator.getMode();
  auto currentShots = aimCalibrator.getCurrentSampleCount();
  auto maxShots = aimCalibrator.getMaxSamples();


  aimCalibrator.appendSample( u, v );
  aimCalibrator.restartCalibration();
  aimCalibrator.cancelCalibration();
  aimCalibrator.saveCalibration();
  */
}
