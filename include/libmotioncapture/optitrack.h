#pragma once
#include "libmotioncapture/motioncapture.h"

namespace libmotioncapture {
  class MotionCaptureOptitrackImpl;

  class MotionCaptureOptitrack : public MotionCapture{
  public:
    MotionCaptureOptitrack(
      const std::string &hostname,
      const std::string& multicast_address = "239.255.42.99",
      int port_command = 1510,
      int port_data = 1511);

    virtual ~MotionCaptureOptitrack();

    const std::string& version() const;

    // implementations for MotionCapture interface
    virtual void waitForNextFrame();
    virtual const std::map<std::string, RigidBody>& rigidBodies() const;
    virtual const PointCloud& pointCloud() const;

    virtual bool supportsRigidBodyTracking() const
    {
      return true;
    }
    virtual bool supportsPointCloud() const
    {
      return true;
    }

  private:
    MotionCaptureOptitrackImpl * pImpl;
  };
}

