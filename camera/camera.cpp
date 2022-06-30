
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <iostream>
#include <vector>



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
	inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1024);
	inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 720);


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


	std::vector<cv::Point2f> centers;
	centers.resize(4);


	cv::Mat frame;
	cv::Mat gray;
	cv::Mat thresh;
	while (true){
		inputVideo >> frame;

		// cv to grey
		cv::cvtColor( frame, gray, cv::COLOR_BGR2GRAY); 

		// threshold
		cv::threshold( gray, thresh, 200, 255, cv::THRESH_BINARY);
		std::vector< std::vector< cv::Point> > contours;

		cv::findContours( thresh, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

		if (contours.size() >= 4) {
			// compute moments
			std::vector< cv::Moments > moments( contours.size() );
			for( size_t i = 0; i < contours.size(); ++i ) {
				moments[i] = cv::moments( contours[i] );
			}

			for( size_t i = 0; i < 4; ++i ) { 
				centers[i] =  cv::Point2f( static_cast<float> ( moments[i].m10 / ( moments[i].m00 + 1e-5)), static_cast<float> ( moments[i].m01 / ( moments[i].m00 + 1e-5)) );
				//std::cout << "center["<<i<<"] = " << centers[i] <<  " area: " << moments[i].m00 <<std::endl;
			}

			// we have 
			cv::Mat rvec, tvec;
			cv::solvePnP(worldPoints, centers, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_AP3P);
			// rvec- is the rotation vector
			std::cout << "rvec: " << rvec << std::endl;	
			// tvec- is the translation vector 
			std::cout << "tvec: " << tvec << std::endl;


			cv::Mat R;
			cv::Rodrigues(rvec, R); // get rotation matrix R ( 3x3 ) from rotation vector 
			std::cout << "R: " << R << std::endl;
			R = R.t(); // inverse
			tvec = -R * tvec; // translation of inverseA == actual camera position
			std::cout << "inverse tvec: " << tvec << std::endl;


			// do intersection
			cv::Vec3f R1,D;
			R1[0] = tvec.at<double>(0);
			R1[1] = tvec.at<double>(1);
			R1[2] = tvec.at<double>(2);

			D[0] =   R.at<double>(0, 2);
			D[1] =   R.at<double>(1, 2);
			D[2] =   R.at<double>(2, 2);


			std::cout << "R: " << R << std::endl;
			std::cout << "D: " << D << std::endl;
			std::cout << "P0: " << P0 << std::endl;
			std::cout << "S1: " << S1 << std::endl;
			std::cout << "S2: " << S2 << std::endl;

			// compute itersection
			float u, v;
			bool hit = intersectRect(R1, D, P0, S1, S2, width, height, u, v);
			std::cout << "U: " << u << " V: " << v << std::endl;
			if( hit ) {
				// TODO: send coordinates to arduino
			}

		} 
		// send -1, -1 to arduino


	}


}
