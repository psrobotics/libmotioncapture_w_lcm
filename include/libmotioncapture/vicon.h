#pragma once
#include "libmotioncapture/motioncapture.h"

// GetSegmentGlobalTranslation
// GetSegmentGlobalRotationQuaternion
// Connect
// IsConnected
// EnableSegmentData
// EnableUnlabeledMarkerData
// EnableMarkerData
// GetVersion
// GetFrame
// GetLatencyTotal,
// GetUnlabeledMarkerCount
// GetUnlabeledMarkerGlobalTranslation

namespace libmotioncapture {

  class MotionCaptureViconImpl;

  class MotionCaptureVicon
    : public MotionCapture
  {
  public:
    MotionCaptureVicon(
      const std::string& hostname,
      bool enableObjects,
      bool enablePointcloud);

    virtual ~MotionCaptureVicon();

    const std::string& version() const;

    // implementations for MotionCapture interface
    virtual const std::map<std::string, Object>& objects() const;
    virtual const Object& objectByName(const std::string& name) const;
    virtual const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud() const;
    virtual const std::vector<LatencyInfo>& latency() const;
    virtual uint64_t timeStamp() const;

    virtual bool supportsObjectTracking() const;
    virtual bool supportsLatencyEstimate() const;
    virtual bool supportsPointCloud() const;
    virtual bool supportsTimeStamp() const;

  private:
    MotionCaptureViconImpl* pImpl;
  };

} // namespace libobjecttracker


