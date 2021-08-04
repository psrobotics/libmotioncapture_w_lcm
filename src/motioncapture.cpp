#include "libmotioncapture/motioncapture.h"

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

}
