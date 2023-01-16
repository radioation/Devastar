#ifndef _IRCAMERA_H_
#define _IRCAMERA_H_

#include <string>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "config.h"
#include "devastar_common.h"


namespace devastar {

  class IRCam : public PointSourceInf {
    public:
      IRCam();
      bool init(const Configuration& conf, const std::string &cameraCalibrationFilename);
      virtual unsigned int getCenters( std::vector<cv::Point2f>& centers ) const;

      virtual bool isRunning() { return m_isRunning; };
      virtual bool stop();

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
      Configuration m_conf;
      unsigned int m_setCount;

  };
};

#endif //_IRCAMERA_H_
