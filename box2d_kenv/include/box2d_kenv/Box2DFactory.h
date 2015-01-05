#ifndef BOX2DFACTORY_H_
#define BOX2DFACTORY_H_
#include <Eigen/Dense>
#include <geos/io/WKTReader.h>
#include "box2d_kenv.hh"

class b2PolygonShape;
class b2World;

namespace geos { namespace geom {
class Polygon;
}
}

namespace YAML {
class Node;
}

namespace box2d_kenv {

class Box2DFactory {
public:
    Box2DFactory(Box2DWorldPtr const &world);

    Box2DBodyPtr CreateBody(std::string const &name,
                            YAML::Node const &node);

    Box2DBodyPtr CreateEmptyBody(std::string const &name);

private:
    Box2DWorldPtr world_;
    b2World *b2_world_;
    geos::io::WKTReader wkt_reader_;

    void SetZero(Box2DLinkPtr const &link,
                 Eigen::Affine2d const &parent_transform);

    Box2DLinkPtr CreateLink(Box2DBodyPtr const &parent_body,
                            YAML::Node const &node);

    Box2DJointPtr CreateJoint(Box2DLinkPtr const &parent_link,
                              Box2DLinkPtr const &child_link,
                              YAML::Node const &node);

    std::vector<b2PolygonShape> ConvertGeometry(
        geos::geom::Polygon const &geom, Eigen::Affine2d const &transform);
};

}

#endif
