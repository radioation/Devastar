
#include <vector>
#include <opencv2/opencv.hpp>

#include "devastar_common.h"


bool getPnPIntersection(const std::vector< cv::Point3f>& worldPoints, 
    const std::vector<cv::Point2f> centers,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    const cv::Vec3f& P0,// Rectangle Origin 
    const cv::Vec3f& S1, // side 1. direction (normalized)
    const cv::Vec3f& S2, // side 2. direction  (normalized)
    const float& S1Len,  // side 1. length
    const float& S2Len,  // side 2. length (height)
    float& u,            // u   // could be 
    float& v	     // v	
    );

void getPerspectiveIntersection( 
		const std::vector<cv::Point2f> centers,
		const std::vector<cv::Point2f>& targetVertices,
		std::vector<cv::Point2f> srcPoints,
		float& u, 
		float& v 
		);




