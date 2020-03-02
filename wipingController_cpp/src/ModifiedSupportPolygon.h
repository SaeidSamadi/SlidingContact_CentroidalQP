#pragma once

#include "SupportPolygon.h"

struct ModifiedSupportPolygon : public SupportPolygon
{
  ModifiedSupportPolygon();
  void update(const mc_rbdyn::Robots & robots, const sva::ForceVecd& fh);

protected:
  std::string handForceSensorName_ = "RightHandForceSensor";
  std::string rightHandSurfaceName_ = "RightHandPad";
};
