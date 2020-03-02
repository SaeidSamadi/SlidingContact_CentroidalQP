#include "ModifiedSupportPolygon.h"

#include <mc_rtc/logging.h>

#include <geos/version.h>
#include <geos/geom/Point.h>

ModifiedSupportPolygon::ModifiedSupportPolygon() : SupportPolygon() {}

void ModifiedSupportPolygon::update(const mc_rbdyn::Robots & robots, const sva::ForceVecd & fh)
{
  // First compute the standard geometric support polygon
  SupportPolygon::update(robots);

  // Shift it according to the hand force
  //const sva::ForceVecd & fh = robots.robot().forceSensor(handForceSensorName_).worldWrench(robots.robot());
  double fz = fh.force().z();
  double mass = robots.robot().mass();
  // FIXME should not be hardcoded
  double g = 9.81;
  double S = 1 - fz / (mass * g);

  const Eigen::Vector3d & ph = robots.robot().surface(rightHandSurfaceName_).X_0_s(robots.robot()).translation();
  Eigen::Vector3d convexShift = (fz * ph) / (mass * g)  - (ph.z() * fh.force()) / (mass * g) ;

  // Shift every point by the convex shift
  // XXX fixme recreates a whole geometry instead of modifying the existing one,
  // inefficient
  // XXX lot of code duplication also
  auto factory_ = geos::geom::GeometryFactory::getDefaultInstance();
  auto temp = factory_->getCoordinateSequenceFactory()->create((std::size_t)0, 0);

  std::vector<geos::geom::Coordinate> cpoints;
  for(size_t i = 0; i < vertices_.size(); ++i)
  {
    Eigen::Vector3d shiftedVertex = S * vertices_[i] + convexShift;
    vertices_[i] = shiftedVertex;
    cpoints.push_back(geos::geom::Coordinate(shiftedVertex.x(), shiftedVertex.y()));
  }
  cpoints.push_back(cpoints[0]);
  temp->setPoints(cpoints);
  auto linearRing = factory_->createLinearRing(std::move(temp));
#if GEOS_VERSION_MAJOR >= 3 && GEOS_VERSION_MINOR >= 8
  auto p = factory_->createPolygon(std::move(linearRing));
#else
  auto p = factory_->createPolygon(std::move(linearRing), 0);
#endif
  auto hull_ = p->convexHull();
  auto c = hull_->getCentroid();
  centroid_.x() = c->getX();
  centroid_.y() = c->getY();
  centroid_.z() = 0;
}
