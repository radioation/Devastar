
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <iostream>
#include <vector>

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#define BUFFER_SIZE 64
#define BAUDRATE B38400            
#define SERIAL_DEVICE "/dev/ttyACM0"

#define SHOW_IMAGE
//#define SHOW_CALC

// intersect rectangle with a ray
// * Ray is defined as R0 + t * D
// 
// * Rectangle is represented with a corner point P0
// and two vectors ( S1 and S2 ) indicating the length
// of the sides of the  rectangle.

bool intersectRect(const cv::Vec3f& R0, // ray start
		const cv::Vec3f& D, // ray direction 
		const cv::Vec3f& P0,// Rectangle Origin 
		const cv::Vec3f& S1, // side 1. direction
		const cv::Vec3f& S2, // side 2. direction 
		const float& S1Len,  // side 1. length
		const float& S2Len,  // side 2. length
		float& u,            // u 
		float& v	     // v	
		) {

	// compute plane Normal 
	cv::Vec3f N = S1.cross(S2);
	float DdotN = D.dot(N);

	// get point on plane P
	auto a = ((P0 - R0).dot(N)) / D.dot(N);
	// we assume that P = R0 + a * D
	cv::Vec3f P = R0 + a * D;  // Point on Plane

	// make vector out of point on plane and orign
	cv::Vec3f P0P = P - P0;

	// project P0P vector onto the sides.
	u = P0P.dot(S1);
	v = P0P.dot(S2);
	return (u >= 0 && u <= S1Len && v >= 0 && v <= S2Len);

}



int main()
{
	// Read in camera calibration calibration
	cv::FileStorage fs("calib.yml", cv::FileStorage::READ);
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	fs["camera_matrix"] >> cameraMatrix;
	fs["dist_coeffs"] >> distCoeffs;


	// setup videocapture
	cv::VideoCapture inputVideo;
	inputVideo.open(0);
	inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 480);


	// Setup object points
	float width = 1700.0f;
	float height = 790.0f;
	std::vector<cv::Point3f> worldPoints;
	worldPoints.push_back(cv::Point3f(0, 0, 0));
	worldPoints.push_back(cv::Point3f(width, 0, 0));
	worldPoints.push_back(cv::Point3f(0, height, 0));
	worldPoints.push_back(cv::Point3f(width, height, 0));
	// setup rectangle for intersection test
	cv::Vec3f S1, S2, P0;
	P0[0] = worldPoints[0].x;
	P0[1] = worldPoints[0].y;
	P0[2] = worldPoints[0].z;

	S1[0] = worldPoints[1].x;
	S1[1] = worldPoints[1].y;
	S1[2] = worldPoints[1].z;
	S1 = cv::normalize(S1);

	S2[0] = worldPoints[2].x;
	S2[1] = worldPoints[2].y;
	S2[2] = worldPoints[2].z;
	S2 = cv::normalize(S2);


	// setup serial communication
	struct termios serial;
	char buffer[BUFFER_SIZE];

	// not controlling TTY.  
	int fd = open( SERIAL_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);
	if( fd < 0 ) {
		std::cerr << "Unable to open serial" << std::endl;
		perror( SERIAL_DEVICE );
		return -1;
	}
	auto result = tcgetattr( fd, & serial );
	if( result < 0 ) {
		std::cerr << "Unable to get serial attributes" << std::endl;
		return -1;
	}

	// Set up Serial Configuration
	cfmakeraw(&serial);

	serial.c_cflag |= (CLOCAL | CREAD);
	serial.c_iflag &= ~(IXOFF | IXANY);

	serial.c_cc[VMIN] = 0;
	serial.c_cc[VTIME] = 0;

	cfsetispeed(&serial, B9600);
	cfsetospeed(&serial, B9600);

	tcsetattr(fd, TCSANOW, &serial); // Apply configuration	

	// setup processing variables
	std::vector<cv::Point2f> centers;
	centers.resize(4);
	cv::Mat frame;
	cv::Mat gray;
	cv::Mat thresh;
	cv::Mat displayCopy;
	const unsigned char offscreen[] = { 0xFF, 0xFF };
	unsigned char xy[2];

	// look at the cmameras
	cv::Point2f pt;
	while (true){
		inputVideo >> frame;

		// cv to grey
		cv::cvtColor( frame, gray, cv::COLOR_BGR2GRAY); 

		// threshold
		cv::threshold( gray, thresh, 200, 255, cv::THRESH_BINARY);
		std::vector< std::vector< cv::Point> > contours;

		cv::findContours( thresh, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

		if (contours.size() >= 4) {
			// compute moments to get centers.  
			std::vector< cv::Moments > moments( contours.size() );
			for( size_t i = 0; i < contours.size(); ++i ) {
				moments[i] = cv::moments( contours[i] );
			}

			//for( size_t i = 0; i < 4; ++i ) { 
			int current = 0;
			for( size_t i = 0; i < contours.size(); ++i ) {
				// using size cutoff to find 'good' candidates.  Look at other features like circularity, position, etc.
				if( moments[i].m00 > 6 && moments[i].m00 < 250 ) {
					centers[current] =  cv::Point2f( static_cast<float> ( moments[i].m10 / ( moments[i].m00 + 1e-5)), static_cast<float> ( moments[i].m01 / ( moments[i].m00 + 1e-5)) );
//#ifdef SHOW_CALC
					std::cout << "center[" << current << "] = " << centers[current] <<  " area: " << moments[i].m00 <<std::endl;
//#endif
					++current;
					if( current == 4 ) {
						break;
					}
				}
			}

#ifdef SHOW_IMAGE
			// display it
			cv::cvtColor( thresh, displayCopy, cv::COLOR_GRAY2BGR); 
			for( const auto& center : centers ) {
				// draw point on image
				cv::circle( displayCopy, center, 3.0f, cv::Scalar(0,0,255), 2.0f);
			}
			cv::circle( displayCopy, pt, 5.0f, cv::Scalar(0,255,255), 3.0f);

			cv::imshow("points", displayCopy );
			cv::waitKey(1);
#endif


			// rvec- is the rotation vector
			// tvec- is the translation vector 
			cv::Mat rvec, tvec;
			auto solveRet = cv::solvePnP(worldPoints, centers, cameraMatrix, distCoeffs, rvec, tvec, false); //, cv::SOLVEPNP_AP3P);

			if( solveRet ) {
				cv::Mat R;
				cv::Rodrigues(rvec, R); // get rotation matrix R ( 3x3 ) from rotation vector 
				R = R.t(); // inverse
				tvec = -R * tvec; // translation of inverseA == actual camera position



				// compute itersection
				cv::Vec3f R1,D;
				R1[0] = tvec.at<double>(0);
				R1[1] = tvec.at<double>(1);
				R1[2] = tvec.at<double>(2);

				D[0] = R.at<double>(0, 2);
				D[1] = R.at<double>(1, 2);
				D[2] = R.at<double>(2, 2);

				float u, v;
				bool hit = intersectRect(R1, D, P0, S1, S2, width, height, u, v);

#ifdef SHOW_CALC
				std::cout << "rvec: " << rvec << std::endl;	
				std::cout << "tvec: " << tvec << std::endl;
				std::cout << "R: " << R << std::endl;
				std::cout << "inverse tvec: " << tvec << std::endl;

				std::cout << "R: " << R << std::endl;
				std::cout << "D: " << D << std::endl;
				std::cout << "P0: " << P0 << std::endl;
				std::cout << "S1: " << S1 << std::endl;
				std::cout << "S2: " << S2 << std::endl;

#endif

				if( hit ) {
#ifdef SHOW_IMAGE
					std::cout << "U: " << u << " V: " << v << std::endl;
					pt.x = (u/width) * 640.0f;
					pt.y = (v/height) * 480.0f;
					cv::circle( displayCopy, pt, 5.0f, cv::Scalar(0,255,255), 3.0f);
					cv::imshow("points", displayCopy );
					cv::waitKey(1);
#endif
					// from obvservation with delay4Cycles() on arduion
					// X range is 73 to 269 : send 0 through 196
					// Y range is 30 to 250 : send 0 through 220

					xy[0] = (unsigned char)( ( u / width) * 196.0f  );
					xy[1] = (unsigned char)( ( v / height) * 196.0f  );
					auto ret = write( fd, xy, 2 );

					continue;
				}
			}


		} 
		// send -1, -1 to arduino
		auto ret = write( fd, offscreen, 2 );

		}



	}
