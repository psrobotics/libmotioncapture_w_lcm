#include "libmotioncapture/motioncapture.h"
#ifdef ENABLE_VICON
#include "libmotioncapture/vicon.h"
#endif
#ifdef ENABLE_OPTITRACK
#include "libmotioncapture/optitrack.h"
#endif
#ifdef ENABLE_PHASESPACE
#include "libmotioncapture/phasespace.h"
#endif
#ifdef ENABLE_QUALISYS
#include "libmotioncapture/qualisys.h"
#endif
#ifdef ENABLE_VRPN
#include "libmotioncapture/vrpn.h"
#endif

#include "yaml-cpp/yaml.h"

namespace libmotioncapture {

  RigidBody MotionCapture::rigidBodyByName(
      const std::string& name) const
  {
    const auto& obj = rigidBodies();
    const auto iter = obj.find(name);
    if (iter != obj.end()) {
      return iter->second;
    }
    throw std::runtime_error("Rigid body not found!");
  }

  MotionCapture* MotionCapture::connect(const std::string &cfg)
  {
    YAML::Node node = YAML::Load(cfg);

    auto type = node["type"].as<std::string>();

    MotionCapture* mocap = nullptr;

    if (false)
    {
    }
#ifdef ENABLE_VICON
    else if (type == "vicon")
    {
      mocap = new libmotioncapture::MotionCaptureVicon(
        node["hostname"].as<std::string>(),
        node["enable_objects"].as<bool>(true),
        node["enable_pointcloud"].as<bool>(true));
    }
#endif
#ifdef ENABLE_OPTITRACK
    else if (type == "optitrack")
    {
      mocap = new libmotioncapture::MotionCaptureOptitrack(
        node["hostname"].as<std::string>());
    }
#endif
#ifdef ENABLE_PHASESPACE
    else if (type == "phasespace")
    {
      std::string ip;
      int numMarkers;
      nl.getParam("phasespace_ip", ip);
      nl.getParam("phasespace_num_markers", numMarkers);
      std::map<size_t, std::pair<int, int>> cfs;
      cfs[231] = std::make_pair<int, int>(10, 11);
      mocap = new libmotioncapture::MotionCapturePhasespace(ip, numMarkers, cfs);
    }
#endif
#ifdef ENABLE_QUALISYS
    else if (type == "qualisys")
    {
      mocap = new libmotioncapture::MotionCaptureQualisys(
        node["hostname"].as<std::string>(),
        node["port"].as<int>(),
        node["enable_objects"].as<bool>(true),
        node["enable_pointcloud"].as<bool>(true));
    }
#endif
#ifdef ENABLE_VRPN
    else if (type == "vrpn")
    {
      mocap = new libmotioncapture::MotionCaptureVrpn(
        node["hostname"].as<std::string>());
    }
#endif
    else
    {
      throw std::runtime_error("Unknown motion capture type!");
    }

    return mocap;
  }

}
