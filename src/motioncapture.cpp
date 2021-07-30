#include "libmotioncapture/motioncapture.h"

namespace libmotioncapture {

  const Object& MotionCapture::objectByName(
      const std::string& name) const
  {
    const auto& obj = objects();
    const auto iter = obj.find(name);
    if (iter != obj.end()) {
      return iter->second;
    }
    throw std::runtime_error("Object not found!");
  }

}
