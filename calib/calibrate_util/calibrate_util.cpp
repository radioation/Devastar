
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <boost/program_options.hpp>
//#include <boost/filesystem.hpp>
//namespace fs = boost::filesystem;
#include <filesystem>
namespace fs = std::filesystem;


#include <iostream>

namespace po = boost::program_options;


int xSquares = 14;
int ySquares = 12;

int imageWidth = 640;
int imageHeight = 480;

float squareLength = 0.015f; // not super important, won't affect actual calculations
float markerLength = 0.010f; // not important either.


// create aruco objects
cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
cv::aruco::CharucoBoard charucoBoard(cv::Size(xSquares, ySquares), squareLength, markerLength, dictionary);

cv::aruco::CharucoParameters charucoParams;
cv::aruco::CharucoDetector detector(charucoBoard, charucoParams, detectorParams);


bool processFrame(cv::Mat image,
        std::vector< cv::Mat >& allCharucoCorners,
        std::vector< cv::Mat >& allCharucoIds,
        std::vector< cv::Mat >& allImgs,
        std::vector<cv::Point3f>& currentObjectPoints,
        std::vector<cv::Point2f>& currentImagePoints,
        cv::Mat &currentCharucoCorners,
        cv::Mat &currentCharucoIds
        ) {
    cv::Mat imageCopy;

    std::vector< int > ids;
    std::vector< std::vector< cv::Point2f > > corners, rejected;


    // detect charuco board
    detector.detectBoard(image, currentCharucoCorners, currentCharucoIds);

    // draw markers on image
    image.copyTo(imageCopy);
    if (ids.size() > 0) {
        cv::aruco::drawDetectedMarkers(imageCopy, corners);
    }

    if (currentCharucoCorners.total() > 0) {
        cv::aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);
    }
    cv::putText(imageCopy, "Press 'a' to add current frame.",
            cv::Point(12, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
    cv::putText(imageCopy, "Press 'c' to calibrate and exit.",
            cv::Point(12, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
    cv::putText(imageCopy, "Press 'ESC to quit without calibrating.",
            cv::Point(12, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);

    imshow("Calibrator", imageCopy);
    // Match image points
    charucoBoard.matchImagePoints(currentCharucoCorners, currentCharucoIds, currentObjectPoints, currentImagePoints);

    if(currentImagePoints.empty() || currentObjectPoints.empty()) {
        std::cout << "Point matching failed, try again." << std::endl;
        return false;
    }
    return true;
}

int main( int argc, char* argv[] )
{
    bool useCamera = true;
    int calibrationFlags = 0;
    float aspectRatio = 1.0f;


    // filesystem
    std::string calibFilename = "calib.yml";
    std::string imageDir = ".";


    // parse CLI options.
    po::options_description desc("Options");
    desc.add_options()
        ("x_squares,x", po::value<int>(), "Number of board squares in X direction")
        ("y_squares,y", po::value<int>(), "Number of board squares in Y direction")
        ("width,w", po::value<int>(), "width of output images")
        ("height,h", po::value<int>(), "height of output image")
        ("filename,f", po::value<std::string>(), "output calibration filename")
        ("images,i", po::value<std::string>(), "Directory of calibration images")
        ("use_images,u", "Use calibration images from (-i) images folder")
        ;
    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        if (vm.count("use_images")){
            useCamera = false;
        }
        if (vm.count("x_squares")) {
            xSquares = vm["x_squares"].as<int>();
        }
        if (vm.count("y_squares")) {
            ySquares = vm["y_squares"].as<int>();
        }
        if (vm.count("width")) {
            imageWidth = vm["width"].as<int>();
        }
        if (vm.count("height")) {
            imageHeight = vm["height"].as<int>();
        }
        if (vm.count("square_length")) {
            squareLength = vm["square_length"].as<float>();
        }
        if (vm.count("marker_length")) {
            markerLength = vm["marker_length"].as<float>();
        }
        if (vm.count("filename")) {
            calibFilename = vm["filename"].as<std::string>();
        }
        if (vm.count("images")) {
            imageDir = vm["images"].as<std::string>();
        }

        po::notify(vm); // throws on error, so do after help in case
                        // there are any problems
    }
    catch (po::error& e)
    {
        std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
        std::cerr << desc << std::endl;
    } 


    charucoBoard.setLegacyPattern(true); // OpenCV is free, so tolerate interface changes.
    charucoParams.tryRefineMarkers = true;


    // make calibration variables
    std::vector< cv::Mat > allCharucoCorners;
    std::vector< cv::Mat > allCharucoIds;
    std::vector< cv::Mat > allImgs;

    std::vector< std::vector< cv::Point2f > > allImagePoints;
    std::vector< std::vector< cv::Point3f > > allObjectPoints;
    cv::Size imgSize;



    // start grabbing images
    if( useCamera ) {
        // get camera 
        cv::VideoCapture inputVideo; 
        inputVideo.open(0); 
        inputVideo.set( cv::CAP_PROP_FRAME_WIDTH,  imageWidth);
        inputVideo.set( cv::CAP_PROP_FRAME_HEIGHT, imageHeight);
        while (inputVideo.grab()) {
            cv::Mat image;
            std::vector<cv::Point3f> currentObjectPoints;
            std::vector<cv::Point2f> currentImagePoints;
            cv::Mat currentCharucoCorners;
            cv::Mat currentCharucoIds;
            inputVideo.retrieve(image);
            bool retProcess = processFrame( image, allCharucoCorners, allCharucoIds, allImgs,
                    currentObjectPoints, currentImagePoints, currentCharucoCorners, currentCharucoIds );


            char key = (char)cv::waitKey(1);
            if (key == 27 ){
                std::cout << "Quit!" << std::endl;
                exit(0);
            }
            if (key == 'c' ) {
                break;
            }
            if (key == 'a' && retProcess ) {
                std::cout << "Frame Added" << std::endl;

                allCharucoCorners.push_back(currentCharucoCorners);
                allCharucoIds.push_back(currentCharucoIds);
                allImagePoints.push_back(currentImagePoints);
                allObjectPoints.push_back(currentObjectPoints);


                allImgs.push_back(image);
                imgSize = image.size();
            }
        }
    } else {
        // iterate images in dir
        for( fs::directory_iterator iter( imageDir ); iter != fs::directory_iterator(); ++ iter ) {
            std::vector<cv::Point3f> currentObjectPoints;
            std::vector<cv::Point2f> currentImagePoints;
            cv::Mat currentCharucoCorners;
            cv::Mat currentCharucoIds;
            cv::Mat image = cv::imread( iter->path().string() );
            bool retProcess = processFrame( image, allCharucoCorners, allCharucoIds, allImgs,
                    currentObjectPoints, currentImagePoints, currentCharucoCorners, currentCharucoIds );
            if( retProcess ) {
                std::cout << "Frame Added" << std::endl;

                allCharucoCorners.push_back(currentCharucoCorners);
                allCharucoIds.push_back(currentCharucoIds);
                allImagePoints.push_back(currentImagePoints);
                allObjectPoints.push_back(currentObjectPoints);


                allImgs.push_back(image);
                imgSize = image.size();
            }
        }
    }


    if(allCharucoCorners.size() < 4) {
        std::cerr << "Not enough corners for calibration" << std::endl;
        return 0;
    }

    cv::Mat cameraMatrix, distCoeffs;
    std::vector< cv::Mat > rvecs, tvecs;

    if(calibrationFlags & cv::CALIB_FIX_ASPECT_RATIO) {
        cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        cameraMatrix.at< double >(0, 0) = aspectRatio;
    }

    // Calibrate camera using ChArUco
    double repError = calibrateCamera(allObjectPoints, allImagePoints, imgSize, cameraMatrix, distCoeffs,
            cv::noArray(), cv::noArray(), cv::noArray(), cv::noArray(), cv::noArray(), calibrationFlags);





    std::cout << "Rep Error: " << repError << std::endl;

    cv::FileStorage fs( calibFilename, cv::FileStorage::WRITE );
    fs << "camera_matrix" << cameraMatrix;
    fs << "dist_coeffs" << distCoeffs;

}
