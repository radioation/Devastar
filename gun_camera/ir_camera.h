#ifndef _IR_CAMMERA_H_
#define _IR_CAMMERA_H_

#include <string>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "config.h"

namespace devastar {

  class IRCam {
    public:
      IRCam();
      bool init(const std::string &cameraCalibrationFilename);
      void getCenters( std::vector<cv::Point2f>& centers );

      bool isRunning() { return m_isRunning; };
      bool stop();

    private:
      std::vector<cv::Point2f> m_centers;
      float m_frameWidth;
      float m_frameHeight;
      bool m_doColorConversion;

      cv::Mat m_cameraMatrix;
      cv::Mat m_distCoeffs;

      std::thread m_captureThread;
      std::mutex m_captureThreadMutex;;

      bool m_isRunning;

      void captureThread();
      devastar::Configuration m_conf;
  };
};

#endif //_IR_CAMMERA_H_
