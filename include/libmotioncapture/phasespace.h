#pragma once
#include "libmotioncapture/motioncapture.h"

#include <map>

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

  class MotionCapturePhasespaceImpl;

  class MotionCapturePhasespace
    : public MotionCapture
  {
  public:
    MotionCapturePhasespace(
      const std::string& hostname,
      size_t numMarkers,
      const std::map<size_t, std::pair<int, int> >& cfs);

    virtual ~MotionCapturePhasespace();

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
    MotionCapturePhasespaceImpl* pImpl;
  };

} // namespace libobjecttracker


