#pragma once
#include "libmotioncapture/motioncapture.h"

namespace libmotioncapture {
  class MotionCaptureTestImpl;

  class MotionCaptureTest : public MotionCapture{
  public:
    MotionCaptureTest(
      float dt,
      const std::vector<Object>& objects);//,
      // const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);

    virtual ~MotionCaptureTest();

    // implementations for MotionCapture interface
    virtual void waitForNextFrame();
    virtual const std::map<std::string, Object>& objects() const;
    virtual const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud() const;
    virtual const std::vector<LatencyInfo>& latency() const;
    virtual uint64_t timeStamp() const;

    virtual bool supportsObjectTracking() const;
    virtual bool supportsLatencyEstimate() const;
    virtual bool supportsPointCloud() const;
    virtual bool supportsTimeStamp() const;

  private:
    MotionCaptureTestImpl * pImpl;
  };
}

