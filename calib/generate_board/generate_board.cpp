
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>

#include <iostream>

int main()
{
	// setup board
	int xSquares = 10;
	int ySquares = 7;
	int imageWidth = 1773;
	int imageHeight = 1230;
	float squareLength = 0.015;
	float markerLength = 0.010;

	// setup aruco dictionary 
	auto dict = cv::aruco::getPredefinedDictionary( cv::aruco::DICT_4X4_50 );

	// create an aruco board
	cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(xSquares, ySquares, squareLength, markerLength, dict);


	// create a target board and save it to  a file.
	cv::Size outsize(imageWidth, imageHeight);
	cv::Mat img;
	board->draw(outsize, img);
	/*
	cv::namedWindow("ChArUco", cv::WINDOW_AUTOSIZE);
	cv::imshow("ChArUco", img);
	cv::waitKey(0);
	*/

	cv::imwrite( "board.png", img);

}
