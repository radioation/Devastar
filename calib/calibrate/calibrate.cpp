
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>

#include <iostream>

int main()
{
	int calibrationFlags = 0;
	float aspectRatio = 1.0f;
	int xSquares = 10;
	int ySquares = 7;
	int imageWidth = 1773;
	int imageHeight = 1230;
	float squareLength = 0.015f;
	float markerLength = 0.010f;

	auto dictionary = cv::aruco::getPredefinedDictionary( cv::aruco::DICT_4X4_50 );

	cv::Ptr< cv::aruco::DetectorParameters > detectorParams = cv::aruco::DetectorParameters::create();                         

	// create charuco board object
	cv::Ptr<cv::aruco::CharucoBoard> charucoboard =
		cv::aruco::CharucoBoard::create(xSquares, ySquares, squareLength, markerLength, dictionary);
	cv::Ptr<cv::aruco::Board> board = charucoboard.staticCast<cv::aruco::Board>();


	// get camera and show charuco                                                                                             
	// create charuco board object                                                                                             

	// collect data from each frame                                                                                            
	std::vector< std::vector< std::vector< cv::Point2f > > > allCorners;                                                       
	std::vector< std::vector< int > > allIds;                                                                                  
	std::vector< cv::Mat > allImgs;                                                                                            
	cv::Size imgSize;                                                                                                          
	cv::VideoCapture inputVideo;                                                                                               
	inputVideo.open(0);   
	inputVideo.set( cv::CAP_PROP_FRAME_WIDTH, 1024);
	inputVideo.set( cv::CAP_PROP_FRAME_HEIGHT, 720);
	std::cout << "width: " << inputVideo.get( cv::CAP_PROP_FRAME_WIDTH) << "\n";
	std::cout << "height: " << inputVideo.get( cv::CAP_PROP_FRAME_HEIGHT) << "\n";


	while (inputVideo.grab()) {
		cv::Mat image, imageCopy;
		inputVideo.retrieve(image);

		std::vector< int > ids;
		std::vector< std::vector< cv::Point2f > > corners, rejected;

		// detect markers
		cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

		cv::aruco::refineDetectedMarkers(image, board, corners, ids, rejected);

		// interpolate charuco corners
		cv::Mat currentCharucoCorners, currentCharucoIds;
		if (ids.size() > 0)
			cv::aruco::interpolateCornersCharuco(corners, ids, image, charucoboard, currentCharucoCorners,
					currentCharucoIds);

		// draw results
		image.copyTo(imageCopy);
		if (ids.size() > 0) {
			cv::aruco::drawDetectedMarkers(imageCopy, corners);
		}

		if (currentCharucoCorners.total() > 0) {
			cv::aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);
		}

		cv::putText(imageCopy, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
				cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);

		imshow("out", imageCopy);
		char key = (char)cv::waitKey(1);
		if (key == 27) {
			std::cout << "image.col " << image.cols << " image.rows " << image.rows << std::endl;
			break;
		}
		if (key == 'c' && ids.size() > 0) {
			std::cout << "Frame tured" << std::endl;
			allCorners.push_back(corners);
			allIds.push_back(ids);
			allImgs.push_back(image);
			imgSize = image.size();
		}
	}


	if(allIds.size() < 1) {
		std::cerr << "Not enough tures for calibration" << std::endl;
		return 0;
	}

	cv::Mat cameraMatrix, distCoeffs;
	std::vector< cv::Mat > rvecs, tvecs;
	double repError;

	if(calibrationFlags & cv::CALIB_FIX_ASPECT_RATIO) {
		cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
		cameraMatrix.at< double >(0, 0) = aspectRatio;
	}

	// prepare data for calibration
	std::vector< std::vector< cv::Point2f > > allCornersConcatenated;
	std::vector< int > allIdsConcatenated;
	std::vector< int > markerCounterPerFrame;
	markerCounterPerFrame.reserve(allCorners.size());
	for(unsigned int i = 0; i < allCorners.size(); i++) {
		markerCounterPerFrame.push_back((int)allCorners[i].size());
		for(unsigned int j = 0; j < allCorners[i].size(); j++) {
			allCornersConcatenated.push_back(allCorners[i][j]);
			allIdsConcatenated.push_back(allIds[i][j]);
		}
	}

	// calibrate camera using aruco markers
	double arucoRepErr;
	arucoRepErr = cv::aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
			markerCounterPerFrame, board, imgSize, cameraMatrix,
			distCoeffs, cv::noArray(), cv::noArray(), calibrationFlags);



	// prepare data for charuco calibration
	int nFrames = (int)allCorners.size();
	std::vector< cv::Mat > allCharucoCorners;
	std::vector< cv::Mat > allCharucoIds;
	std::vector< cv::Mat > filteredImages;
	allCharucoCorners.reserve(nFrames);
	allCharucoIds.reserve(nFrames);

	for(int i = 0; i < nFrames; i++) {
		// interpolate using camera parameters
		cv::Mat currentCharucoCorners, currentCharucoIds;
		cv::aruco::interpolateCornersCharuco(allCorners[i], allIds[i], allImgs[i], charucoboard,
				currentCharucoCorners, currentCharucoIds, cameraMatrix,
				distCoeffs);

		allCharucoCorners.push_back(currentCharucoCorners);
		allCharucoIds.push_back(currentCharucoIds);
		filteredImages.push_back(allImgs[i]);
	}

	if(allCharucoCorners.size() < 4) {
		std::cerr << "Not enough corners for calibration" << std::endl;
		return 0;
	}

	// calibrate camera using charuco
	repError = cv::aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, imgSize,
			cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags);

	/*
	   bool saveOk =  saveCameraParams(outputFile, imgSize, aspectRatio, calibrationFlags,
	   cameraMatrix, distCoeffs, repError);
	   if(!saveOk) {
	   std::cerr << "Cannot save output file" << std::endl;
	   return 0;
	   }
	   */

	std::cout << "Rep Error: " << repError << std::endl;
	std::cout << "Rep Error Aruco: " << arucoRepErr << std::endl;

	cv::FileStorage fs( "calib.yml", cv::FileStorage::WRITE );
	fs << "camera_matrix" << cameraMatrix;
	fs << "dist_coeffs" << distCoeffs;

}
