#pragma once
#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/Surface.h>

#include <geos/geom/CoordinateSequence.h>
#include <geos/geom/CoordinateSequenceFactory.h>
#include <geos/geom/Geometry.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/Polygon.h>

#include <vector>

struct SupportPolygon
{
  SupportPolygon();
  void setContacts(const std::vector<mc_rbdyn::Contact> & contacts);
  void update(const mc_rbdyn::Robots & robots);
  const std::vector<Eigen::Vector3d> & vertices()
  {
    return vertices_;
  }
  const Eigen::Vector3d & centralPoint() const
  {
    return centroid_;
  }

protected:
  std::vector<mc_rbdyn::Contact> contacts_;
  std::vector<Eigen::Vector3d> vertices_;
  Eigen::Vector3d centroid_;
};
