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
    const std::vector<RigidBody>& objects)//,
    // const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud)
  {
    pImpl = new MotionCaptureTestImpl;
    pImpl->dt = dt;
    for (const auto& obj : objects) {
      rigidBodies_[obj.name()] = obj;
    }
    Eigen::Quaternionf q(0,0,0,1); 
    rigidBodies_["test"] = RigidBody("test", Eigen::Vector3f(0,1,2), q);
    // pointcloud_ = pointCloud;
  }

  void MotionCaptureTest::waitForNextFrame()
  {
    std::this_thread::sleep_for(std::chrono::milliseconds((int)(pImpl->dt * 1000)));
    latencies_.clear();
    // pointcloud_->clear();
    latencies_.clear();
  }

  const std::map<std::string, RigidBody>& MotionCaptureTest::rigidBodies() const
  {
    return rigidBodies_;
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

  bool MotionCaptureTest::supportsRigidBodyTracking() const
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

