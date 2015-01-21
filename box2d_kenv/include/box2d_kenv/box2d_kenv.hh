#ifndef BOX2D_KENV_HH_
#define BOX2D_KENV_HH_
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

namespace geos {
namespace geom {

class Geometry;

}
}

namespace box2d_kenv {

class Box2DWorld;
typedef boost::shared_ptr<Box2DWorld> Box2DWorldPtr;
typedef boost::weak_ptr<Box2DWorld> Box2DWorldWeakPtr;

class Box2DBody;
typedef boost::shared_ptr<Box2DBody> Box2DBodyPtr;
typedef boost::weak_ptr<Box2DBody> Box2DBodyWeakPtr;

class Box2DLink;
typedef boost::shared_ptr<Box2DLink> Box2DLinkPtr;
typedef boost::weak_ptr<Box2DLink> Box2DLinkWeakPtr;

class Box2DSensor;
typedef boost::shared_ptr<Box2DSensor> Box2DSensorPtr;
typedef boost::weak_ptr<Box2DSensor> Box2DSensorWeakPtr;

class Box2DJoint;
typedef boost::shared_ptr<Box2DJoint> Box2DJointPtr;
typedef boost::weak_ptr<Box2DJoint> Box2DJointWeakPtr;

typedef boost::shared_ptr<geos::geom::Geometry> GeometryPtr;
typedef boost::shared_ptr<geos::geom::Geometry const> GeometryConstPtr;

}

#endif
