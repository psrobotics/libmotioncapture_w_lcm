#pragma once
#include "libmotioncapture/motioncapture.h"

namespace libmotioncapture {

  class MotionCaptureQualisysImpl;

  class MotionCaptureQualisys
    : public MotionCapture
  {
  public:
    MotionCaptureQualisys(
      const std::string& hostname,
      int basePort,
      bool enableObjects,
      bool enablePointcloud);

    virtual ~MotionCaptureQualisys();

    const std::string& version() const;

    // implementations for MotionCapture interface
    virtual const std::map<std::string, RigidBody>& rigidBodies() const;
    virtual const RigidBody& rigidBodyByName(const std::string& name) const;
    virtual const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud() const;
    virtual const std::vector<LatencyInfo>& latency() const;
    virtual uint64_t timeStamp() const;

    virtual bool supportsRigidBodyTracking() const;
    virtual bool supportsLatencyEstimate() const;
    virtual bool supportsPointCloud() const;
    virtual bool supportsTimeStamp() const;

  private:
    MotionCaptureQualisysImpl* pImpl;
  };

} // namespace libobjecttracker


