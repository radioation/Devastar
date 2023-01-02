#ifndef _IR_CAMMERA_H_
#define _IR_CAMMERA_H_

#include <string>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "config.h"

namespace devastar {

  class DFRobot {
    public:
      DFRobot();
      bool init(const Configuration& conf );
      void getCenters( std::vector<cv::Point2f>& centers );

      bool isRunning() { return m_isRunning; };
      bool stop();

    private:
      std::vector<cv::Point2f> m_centers;

      std::thread m_captureThread;
      std::mutex m_captureThreadMutex;

      std::string m_deviceName;
      int m_devFile;
      bool m_isRunning;

      void captureThread();
      Configuration m_conf;
  };
};

#endif //_IR_CAMMERA_H_
