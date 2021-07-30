#include "libmotioncapture/testmocap.h"

#include <thread>

#include <pcl/common/io.h>

namespace libmotioncapture {

  class MotionCaptureTestImpl{
  public:
    MotionCaptureTestImpl()
    {
    }

  public:
    float dt;
  };

  MotionCaptureTest::MotionCaptureTest(
    float dt,
    const std::vector<Object>& objects)//,
    // const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud)
  {
    pImpl = new MotionCaptureTestImpl;
    pImpl->dt = dt;
    for (const auto& obj : objects) {
      objects_[obj.name()] = obj;
    }
    Eigen::Quaternionf q(0,0,0,1); 
    objects_["test"] = Object("test", Eigen::Vector3f(0,1,2), q);
    // pointcloud_ = pointCloud;
  }

  void MotionCaptureTest::waitForNextFrame()
  {
    std::this_thread::sleep_for(std::chrono::milliseconds((int)(pImpl->dt * 1000)));
    latencies_.clear();
    // pointcloud_->clear();
    latencies_.clear();
  }

  const std::map<std::string, Object>& MotionCaptureTest::objects() const
  {
    return objects_;
  }

  const pcl::PointCloud<pcl::PointXYZ>::Ptr MotionCaptureTest::pointCloud() const
  {
    return pointcloud_;
  }

  const std::vector<libmotioncapture::LatencyInfo>& MotionCaptureTest::latency() const
  {
    return latencies_;
  }

  uint64_t MotionCaptureTest::timeStamp() const
  {
    return 0;
  }

  MotionCaptureTest::~MotionCaptureTest()
  {
    delete pImpl;
  }

  bool MotionCaptureTest::supportsObjectTracking() const
  {
    return true;
  }

  bool MotionCaptureTest::supportsLatencyEstimate() const
  {
    return false;
  }

  bool MotionCaptureTest::supportsPointCloud() const
  {
    return true;
  }

  bool MotionCaptureTest::supportsTimeStamp() const
  {
    return false;
  }
}

