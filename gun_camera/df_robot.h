#ifndef _DF_ROBOT_H_
#define _DF_ROBOT_H_

#include <string>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "config.h"
#include "devastar_common.h"


namespace devastar {

  class DFRobot : public PointSourceInf {
    public:
      DFRobot();
      bool init(const Configuration& conf );
      virtual unsigned int getCenters( std::vector<cv::Point2f>& centers ) const;

      virtual bool isRunning() { return m_isRunning; };
      virtual bool stop();

    private:
      std::vector<cv::Point2f> m_centers;

      std::thread m_captureThread;
      std::mutex m_captureThreadMutex;

      std::string m_deviceName;
      int m_devFile;
      bool m_isRunning;

      void captureThread();
      Configuration m_conf;
      unsigned int m_setCount;
  };
};

#endif //_DF_ROBOT_H_
