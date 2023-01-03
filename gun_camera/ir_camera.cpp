
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <chrono>
#include <bitset>
#include <gpiod.h>


#include <iostream>
#include <vector>

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>


#include "compute.h"
#include "config.h"
#include "devastar_common.h"
#include "ir_camera.h"


#include <filesystem>
namespace fs = std::filesystem;


using namespace devastar;


IRCam::IRCam() :
	m_frameWidth(0.0f),
	m_frameHeight(0.0f),
	m_doColorConversion(false),
	m_isRunning(false)
{
}

bool IRCam::init(const Configuration& conf, const std::string& cameraCalibrationFilename )
{
	m_conf = conf;
	m_frameWidth = conf.frameWidth;
	m_frameHeight = conf.frameHeight;
	// Read in camera calibration calibration
	fs::path calibPath( cameraCalibrationFilename ); 
	if( fs::exists(calibPath)) {
		cv::FileStorage fileStorage(calibPath, cv::FileStorage::READ);
		fileStorage["camera_matrix"] >> m_cameraMatrix;
		fileStorage["dist_coeffs"] >> m_distCoeffs;

		// check if valid?
		m_captureThread = std::thread( &IRCam::captureThread, this );
		return true;
	}    
	return false;
}


void IRCam::getCenters( std::vector<cv::Point2f>& centers ) const {
	std::transform(m_centers.begin(), m_centers.end(), std::back_inserter(centers), 
			[](auto e){ return e; });   
}

bool IRCam::stop() {
	if( m_isRunning ) {
		m_isRunning = false;
		if( m_captureThread.joinable() ) {
			m_captureThread.join();
		}
		return true;
	}
	return false;
}


void IRCam::captureThread() {
	// setup videocapture
	cv::VideoCapture inputVideo;
	inputVideo.open(0); // assuming default camera, TODO: make configurable
	inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, int(m_frameWidth));
	inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, int(m_frameHeight));
	// buffer size may or may not be supported 
	std::cout << "CAP_PROP_BUFFERSIZE : " << inputVideo.set(cv::CAP_PROP_BUFFERSIZE,  1) << std::endl;

	// setup 
	m_centers.resize(4);
	cv::Mat frame;
	cv::Mat gray;
	cv::Mat thresh;
	cv::Mat displayCopy;
	// DF_ROBOT?
	std::vector<cv::Point2f> undistortCenters;
	auto noArray = cv::noArray();
	undistortCenters.resize(4);

	// screen center point
	std::vector<cv::Point2f> srcPoints(1);
	srcPoints[0].x = float(m_frameWidth/2);
	srcPoints[0].y = float(m_frameHeight/2);

	m_isRunning = true;
	cv::Point2f pt;
	while (true) {
#ifdef SHOW_3D
		glutMainLoopEvent(); // process events to keep GL cycling
#endif

		// Process Video from camera
#ifdef SHOW_TIME
		startTime = std::chrono::steady_clock::now();
#endif
		if( m_doColorConversion ) {
			inputVideo >> frame;
		} else {
			inputVideo >> gray;
		}
#ifdef SHOW_TIME
		endTime = std::chrono::steady_clock::now();
		std::cout << "ELAPSED TIME>> frame grab: " << float(std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count()) / 1000.0f << "\n"; 
#endif
		// convert to grayscale
		if( m_doColorConversion ) {
#ifdef SHOW_TIME
			startTime = std::chrono::steady_clock::now();
#endif
			cv::cvtColor( frame, gray, cv::COLOR_BGR2GRAY); 
#ifdef SHOW_TIME
			endTime = std::chrono::steady_clock::now();
			std::cout << "ELAPSED TIME>> cvtColor(): " << float(std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count()) / 1000.0f << "\n"; 
#endif
		}


		// threshold grayscale image to eliminate dark blobs
#ifdef SHOW_TIME
		startTime = std::chrono::steady_clock::now();
#endif
		cv::threshold( gray, thresh, m_conf.irThreshold, 255, cv::THRESH_BINARY);
#ifdef SHOW_TIME
		endTime = std::chrono::steady_clock::now();
		std::cout << "ELAPSED TIME>> cv::threshold(): " << float(std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count()) / 1000.0f << "\n"; 
#endif
		std::vector< std::vector< cv::Point> > contours;
		// look for candidate IR lights in the remaining blobs
#ifdef SHOW_TIME
		startTime = std::chrono::steady_clock::now();
#endif
		cv::findContours( thresh, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
#ifdef SHOW_TIME
		endTime = std::chrono::steady_clock::now();
		std::cout << "ELAPSED TIME>> cv::findContours(): " << float(std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count()) / 1000.0f << "\n"; 
#endif

#ifdef SHOW_TIME
		startTime = std::chrono::steady_clock::now();
#endif
		float u = -1.0f, v = -1.0f;
		bool solveRet = false;
		// Look for screen intersection if we have at least 4 points
		if (contours.size() >= 4) {
#ifdef SHOW_TIME
			startTime = std::chrono::steady_clock::now();
#endif
			// compute moments to get centers.  
			std::vector< cv::Moments > moments( contours.size() );
			for( size_t i = 0; i < contours.size(); ++i ) {
				moments[i] = cv::moments( contours[i] );
			}

			int centerCount = 0;
			for( size_t i = 0; i < contours.size(); ++i ) {
				// using size cutoff to find 'good' candidates.  Look at other features like circularity, position, etc.
				if( moments[i].m00 > m_conf.minBlobSize && moments[i].m00 < m_conf.maxBlobSize ) {

					if( centerCount < 4 ) {
						m_centers[centerCount] = cv::Point2f( static_cast<float> ( moments[i].m10 / ( moments[i].m00 + 1e-5)), static_cast<float> ( moments[i].m01 / ( moments[i].m00 + 1e-5)) );

					}
					++centerCount;

					if( centerCount == 4 ) {
						break;
					}
				}
			}
#ifdef SHOW_TIME
			endTime = std::chrono::steady_clock::now();
			std::cout << "ELAPSED TIME>> get centers from moments: " << float(std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count()) / 1000.0f << "\n"; 
#endif



			// order centers for later procesing
			// use simplest case for now -> only calculate hit if count is 4 
			// TODO: Also check if "n >= 4".  potentially look at some distance
			// minimizaion check.
			if( centerCount == 4 ) {
#ifdef SHOW_TIME
				startTime = std::chrono::steady_clock::now();
#endif
				size_t a=0;
				size_t b=1;
				size_t c=2;
				size_t d=3;
				size_t ab1, ab2, cd1,cd2;
				size_t i1, i2, i3, i4, m1, m2;

				// sort along X
				// split into 2 sets {a,b}  and {c,d}
				if( m_centers[a].x < m_centers[b].x ) {
					ab1 = a;
					ab2 = b;
				} else {
					ab1 = b;
					ab2 = a;
				}

				if( m_centers[c].x < m_centers[d].x ) {
					cd1 = c;
					cd2 = d;
				} else {
					cd1 = d;
					cd2 = c;
				}

				if( m_centers[ab1].x < m_centers[cd1].x ) {
					i1 = ab1;
					m1 = cd1;
				} else {
					i1 = cd1;
					m1 = ab1;
				}

				if( m_centers[ab2].x > m_centers[cd2].x ) {
					i4 = ab2;
					m2 = cd2;
				} else {
					i4 = cd2;
					m2 = ab2;
				}

				if( m_centers[m1].x < m_centers[m2].x ) {
					i2 = m1;
					i3 = m2;
				} else {
					i2 = m2;
					i3 = m1;
				}

				cv::Point2f pt1 = m_centers[i1];
				cv::Point2f pt2 = m_centers[i2];
				cv::Point2f pt3 = m_centers[i3];
				cv::Point2f pt4 = m_centers[i4];

				// compare higehts of first two and last two
				if( pt1.y < pt2.y ) {
					m_centers[0] = pt1;
					m_centers[2] = pt2;
				} else {
					m_centers[0] = pt2;
					m_centers[2] = pt1;
				}
				if( pt3.y < pt4.y ) {
					m_centers[1] = pt3;
					m_centers[3] = pt4;
				} else {
					m_centers[1] = pt4;
					m_centers[3] = pt3;
				}

#ifdef SHOW_TIME
				endTime = std::chrono::steady_clock::now();
				std::cout << "ELAPSED TIME>> sort centers: " << float(std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count()) / 1000.0f << "\n"; 
#endif


#ifdef SHOW_IMAGE
				// display it
				//cv::cvtColor( thresh, displayCopy, cv::COLOR_GRAY2BGR); 
				displayCopy = frame.clone();
				for( int i=0; i < m_centers.size(); ++ i ) {
					// draw point on image
					cv::Scalar color( 0,0,255 );
					if( i == 1 ) {
						color = cv::Scalar( 0,255,0 );
					} else if( i== 2 ) {
						color = cv::Scalar( 255,0,0 );
					} else if( i== 3 ) {
						color = cv::Scalar( 255,255,0 );
					}
					cv::circle( displayCopy, m_centers[i], 10.0f, color, 2.0f);
				}
				//cv::circle( displayCopy, pt, 5.0f, cv::Scalar(0,255,255), 3.0f);

				cv::imshow("points", displayCopy );
				cv::waitKey(1);
#endif

			}// while(true)
		}
	}

}

