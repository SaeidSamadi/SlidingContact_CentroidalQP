#include "SupportPolygon.h"

#include <mc_rtc/logging.h>

#include <geos/version.h>
#include <geos/geom/Point.h>

SupportPolygon::SupportPolygon()
{
}

void SupportPolygon::setContacts(const std::vector<mc_rbdyn::Contact> & contacts)
{
  if(contacts.empty())
  {
    LOG_ERROR("Cannot compute a support polygon with zero contacts");
  }
  contacts_ = contacts;
}

void SupportPolygon::update(const mc_rbdyn::Robots & robots)
{
  if(contacts_.empty())
  {
    LOG_WARNING("SupportPolygon has no contact");
    return;
  }
  // Compute support polygon
  std::vector<sva::PTransformd> points;
  for(const auto & contact : contacts_)
  {
    const auto & surface = const_cast<mc_rbdyn::Surface &>(*contact.r1Surface());
    // Points in body frame
    const auto & pts = surface.points();
    const sva::PTransformd & X_0_b = robots.robot(contact.r1Index()).bodyPosW(surface.bodyName());
    for(const auto & p : pts)
    {
      sva::PTransformd X_0_p = p * X_0_b;
      points.push_back(X_0_p.translation());
    }
  }
  auto factory_ = geos::geom::GeometryFactory::getDefaultInstance();
  auto temp = factory_->getCoordinateSequenceFactory()->create((std::size_t)0, 0);

  std::vector<geos::geom::Coordinate> cpoints;
  for(const auto & p : points)
  {
    cpoints.push_back(geos::geom::Coordinate(p.translation().x(), p.translation().y()));
  }
  cpoints.push_back(cpoints[0]);
  temp->setPoints(cpoints);
  auto g = factory_->createLinearRing(std::move(temp));
#if GEOS_VERSION_MAJOR >= 3 && GEOS_VERSION_MINOR >= 8
  auto p = factory_->createPolygon(std::move(g));
#else
  auto p = factory_->createPolygon(std::move(g), 0);
#endif
  auto hull_ = p->convexHull();
  auto seq = hull_->getCoordinates();
  vertices_.resize(seq->size());
  for(size_t i = 0; i < seq->size(); ++i)
  {
    auto coord = seq->getAt(i);
    vertices_[i] = Eigen::Vector3d{coord.x, coord.y, 0.};
  }
  auto c = hull_->getCentroid();
  centroid_.x() = c->getX();
  centroid_.y() = c->getY();
  centroid_.z() = 0;
}
