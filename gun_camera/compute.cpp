#include "compute.h"


#include <chrono>

#include <experimental/string_view>

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



void computeUV(const cv::Vec3f& R0, // ray start
    const cv::Vec3f& D, // ray direction 
    const cv::Vec3f& P0,// Rectangle Origin 
    const cv::Vec3f& S1, // side 1. direction
    const cv::Vec3f& S2, // side 2. direction 
    const float& S1Len,  // side 1. length
    const float& S2Len,  // side 2. length
    float& u,            // u   // could be 
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
  //return (u >= 0 && u <= S1Len && v >= 0 && v <= S2Len);

}




bool getPnPIntersection(const std::vector< cv::Point3f>& worldPoints, 
    const std::vector<cv::Point2f> centers,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    const cv::Vec3f& P0,// Rectangle Origin 
    const cv::Vec3f& S1, // side 1. direction
    const cv::Vec3f& S2, // side 2. direction 
    const float& S1Len,  // side 1. length  
    const float& S2Len,  // side 2. length
    float& u,            // u   // could be 
    float& v	     // v	
    ) {
  cv::Mat rvec, tvec;
#ifdef SHOW_TIME
  auto startTime = std::chrono::steady_clock::now();
#endif
  bool solveRet = cv::solvePnP(worldPoints, centers, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_AP3P);
#ifdef SHOW_TIME
  auto endTime = std::chrono::steady_clock::now();
  std::cout << "ELAPSED TIME>> solvePnP(): " << float(std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count()) / 1000.0f << "\n"; 
#endif
  bool hit = false;
  if( solveRet ) {

#ifdef SHOW_TIME
    startTime = std::chrono::steady_clock::now();
#endif
    cv::Mat R;
    cv::Rodrigues(rvec, R); // get rotation matrix R ( 3x3 ) from rotation vector 
    R = R.t(); // inverse
    tvec = -R * tvec; // translation of inverseA == actual camera position

    // compute itersection
    cv::Vec3f Ray0,D;
    Ray0[0] = tvec.at<double>(0);
    Ray0[1] = tvec.at<double>(1);
    Ray0[2] = tvec.at<double>(2);

    D[0] = R.at<double>(0, 2);
    D[1] = R.at<double>(1, 2);
    D[2] = R.at<double>(2, 2);

    //hit = intersectRect(Ray0, D, P0, S1, S2, irWidth, irHeight, u, v);
    computeUV(Ray0, D, P0, S1, S2, S1Len, S2Len, u, v);
#ifdef SHOW_TIME
    endTime = std::chrono::steady_clock::now();
    std::cout << "ELAPSED TIME>>  R/tvec and computeUV(): " << float(std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count()) / 1000.0f << "\n"; 
    std::cout << " AP3P  U: " << u << " V: " << v << std::endl;
#endif


#ifdef SHOW_CALC
    std::cout << "centers[0]: " << centers[0] << std::endl;
    std::cout << "centers[1]: " << centers[1] << std::endl;
    std::cout << "centers[2]: " << centers[2] << std::endl;
    std::cout << "centers[3]: " << centers[3] << std::endl;
    std::cout << "rvec: " << rvec << std::endl;	
    std::cout << "tvec: " << tvec << std::endl;
    std::cout << "R: " << R << std::endl;
    std::cout << "inverse tvec: " << tvec << std::endl;

    std::cout << "Ray0: " << Ray0 << std::endl;
    std::cout << "D: " << D << std::endl;
    std::cout << "P0: " << P0 << std::endl;
    std::cout << "S1: " << S1 << std::endl;
    std::cout << "S2: " << S2 << std::endl;
    std::cout << "irWidth: " << irWidth  << std::endl;
    std::cout << "irHeight: " << irHeight << std::endl;
    std::cout << "U: " << u << " V: " << v << " hit: " << hit << std::endl;
#endif

    return true;
  }
  return false;

}



void getPerspectiveIntersection( 
    const std::vector<cv::Point2f> centers,
    const std::vector<cv::Point2f>& targetVertices,
    std::vector<cv::Point2f> srcPoints,
    float& u, 
    float& v 
    ) {

#ifdef SHOW_TIME
  auto startTime = std::chrono::steady_clock::now();
#endif
  cv::Mat rotationMatrix = getPerspectiveTransform(centers, targetVertices);
#ifdef SHOW_TIME
  auto endTime = std::chrono::steady_clock::now();
  std::cout << "ELAPSED TIME>> solve getPerspectiveTransform(): " << float(std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count()) / 1000.0f << "\n"; 
#endif
#ifdef SHOW_TIME
  startTime = std::chrono::steady_clock::now();
#endif
  std::vector<cv::Point2f> dstCenters;
  cv::perspectiveTransform(srcPoints, dstCenters, rotationMatrix);
#ifdef SHOW_TIME
  endTime = std::chrono::steady_clock::now();
  std::cout << "ELAPSED TIME>> solve perspectiveTransform(): " << float(std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count()) / 1000.0f << "\n"; 
  std::cout << "dstCenters[0]" << dstCenters[0] <<std::endl;
  std::cout << "centers[0]: " << centers[0] << std::endl;
  std::cout << "centers[1]: " << centers[1] << std::endl;
  std::cout << "centers[2]: " << centers[2] << std::endl;
  std::cout << "centers[3]: " << centers[3] << std::endl;
  std::cout << "srcPoints[0]: " << srcPoints[0] << std::endl;
#endif

}




