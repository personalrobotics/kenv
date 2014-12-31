#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Dynamics/Joints/b2RevoluteJoint.h>
#include <geos/geom/Geometry.h>
#include <geos/io/WKTReader.h>
#include <yaml-cpp/yaml.h>
#include "geometry_utils.h"
#include "yaml_utils.h"
#include "Box2DBody.h"
#include "Box2DLink.h"
#include "Box2DFactory.h"
#include "Box2DJoint.h"
#include "Box2DWorld.h"

using boost::make_shared;

namespace box2d_kenv {

Box2DFactory::Box2DFactory(Box2DWorldPtr const &world)
    : world_(world)
    , b2_world_(world->b2_world())
    , wkt_reader_(geos::geom::GeometryFactory::getDefaultInstance())
{
    BOOST_ASSERT(world);
}

Box2DBodyPtr Box2DFactory::CreateBody(std::string const &name,
                                      YAML::Node const &node)
{
    Box2DBodyPtr const body = make_shared<Box2DBody>(world_, name);
    body->root_link_ = CreateLink(body, node);

    // Set the links' initial poses. Otherwise, the links could overlap and
    // generate a large transient force.
    SetZero(body->root_link_, Eigen::Affine2d::Identity());

    return body;
}

Box2DLinkPtr Box2DFactory::CreateLink(Box2DBodyPtr const &parent_body,
                                      YAML::Node const &node)
{
    using namespace box2d_kenv::util;

    typedef boost::shared_ptr<geos::geom::Geometry> GeometryPtr;

    BOOST_ASSERT(parent_body);

    std::string name;
    std::string geometry_wkt;
    Eigen::Affine2d relative_pose;

    node["name"] >> name;
    node["relative_geometry"] >> geometry_wkt;
    node["relative_pose"] >> relative_pose;

    // Deserialize the link's geometry (from WKT). To simplify the conversion,
    // we'll only support polygons. This constraint could be relaxed.
    GeometryPtr const geometry(wkt_reader_.read(geometry_wkt));
    if (geometry->getGeometryTypeId() != geos::geom::GEOS_POLYGON) {
        throw std::runtime_error(boost::str(
            boost::format("Geometry for link '%s' has type '%s'; only polygons"
                          " are supported.")
                % name % geometry->getGeometryType()
        ));
    }
    geos::geom::Polygon const &polygon
        = dynamic_cast<geos::geom::Polygon const &>(*geometry);

    // Construct Box2D fixtures to represent the geometry. Box2D places several
    // limitations on fixtures, so one GEOS polygon may result in several Box2D
    // fixtures. See ConverGeometry for more information.
    b2BodyDef b2_bodydef;
    b2_bodydef.active = true;
    b2_bodydef.awake = true;
    b2_bodydef.allowSleep = true;
    b2_bodydef.type = b2_dynamicBody;

    b2Body *b2_body = b2_world_->CreateBody(&b2_bodydef);
    std::vector<b2PolygonShape> const b2_shapes
        = ConvertGeometry(polygon, relative_pose);

    BOOST_FOREACH (b2PolygonShape const &b2_shape, b2_shapes) {
        // TODO: Load these properties from YAML.
        b2FixtureDef b2_fixturedef;
        b2_fixturedef.density = 1000.;
        b2_fixturedef.friction = 0.5; // TODO: What does this mean?
        b2_fixturedef.restitution = 0.0; // TODO: What does this mean?
        b2_fixturedef.shape = &b2_shape;

        b2_body->CreateFixture(&b2_fixturedef);
    }

    // Create this link.
    Box2DLinkPtr const link = make_shared<Box2DLink>(parent_body, name, b2_body);
    // Recursively construct this link's children.
    for (size_t i = 0; i < node["joints"].size(); ++i) {
        YAML::Node const &joint_node = node["joints"][i];

        // Recursively construct the child link.
        YAML::Node const &child_node = joint_node["child_link"];
        Box2DLinkPtr const child_link = CreateLink(parent_body, child_node);

        // Connect the child link to this link with a joint.
        Box2DJointPtr const joint = CreateJoint(link, child_link, joint_node);
        link->AddChildJoint(joint);
    }

    return link;
}

Box2DJointPtr Box2DFactory::CreateJoint(Box2DLinkPtr const &parent_link,
                                        Box2DLinkPtr const &child_link,
                                        YAML::Node const &node)
{
    using namespace ::box2d_kenv::util;

    BOOST_ASSERT(parent_link);
    BOOST_ASSERT(child_link);

    double const scale = world_->scale();

    Eigen::Affine2d relative_origin;
    node["relative_origin"] >> relative_origin;

    b2RevoluteJointDef b2_jointdef;
    b2_jointdef.collideConnected = false;
    b2_jointdef.bodyA = parent_link->b2_body();
    b2_jointdef.bodyB = child_link->b2_body();
    b2_jointdef.localAnchorA = b2Vec2(
        scale * relative_origin.translation()[0], 
        scale * relative_origin.translation()[1]
    );
    b2_jointdef.localAnchorB = b2Vec2(0., 0.);

    // Set the initial angle of the joint. By convention, this is the x-axis.
    Eigen::Rotation2Dd rotation(0.);
    rotation.fromRotationMatrix(relative_origin.rotation());
    b2_jointdef.referenceAngle = rotation.angle();
    
    // TODO: Read joint limits from YAML.
    b2_jointdef.enableLimit = false;
    b2_jointdef.lowerAngle = -b2_pi;
    b2_jointdef.upperAngle =  b2_pi;

    // TODO: Read the maximum motor torque from YAML.
    b2_jointdef.enableMotor = true;
    b2_jointdef.maxMotorTorque = 10.;

    b2RevoluteJoint *b2_joint = static_cast<b2RevoluteJoint *>(
        b2_world_->CreateJoint(&b2_jointdef));

    return boost::make_shared<Box2DJoint>(parent_link, child_link, b2_joint);
}

std::vector<b2PolygonShape> Box2DFactory::ConvertGeometry(
        geos::geom::Polygon const &geom, Eigen::Affine2d const &transform)
{
    using namespace ::box2d_kenv::util;

    typedef boost::shared_ptr<geos::geom::Polygon> PolygonPtr;

    double const scale = world_->scale();
    double const weld_distance = 0.5 * b2_linearSlop / scale;

    // Transform the geometry into the link frame.
    PolygonPtr geom_clone(static_cast<geos::geom::Polygon *>(geom.clone()));
    AffineTransformFilter transform_filter(transform);
    geom_clone->apply_rw(&transform_filter);

    // First, convert the GEOS polygon returned by polygonal_kenv to a CGAL
    // polygon datatype. We use CGAL to perform the convex decomposition.
    Polygon_2 const input_polygon = ConvertPolygonGEOStoCGAL(*geom_clone, scale,
                                                             weld_distance);
    Polygon_list const convex_polygons = DecomposeCGALPolygon(input_polygon);

    // Next, split each polygon into chunks that are small enough to be used in
    // Box2D. Box2D imposes a maximum number of vertices on a polygon. Convert
    // the output to Box2D polygons.
    std::vector<b2PolygonShape> b2_polygons;

    BOOST_FOREACH (Polygon_2 const &polygon, convex_polygons) {
        Polygon_list split_polygons;
        RecursivelySplitCGALPolygon(polygon, b2_maxPolygonVertices, &split_polygons);

        BOOST_FOREACH (Polygon_2 const &polygon, split_polygons) {
            b2PolygonShape const b2_polygon = ConvertPolygonCGALtoBox2D(polygon);
            b2_polygons.push_back(b2_polygon);
        }
    }

    return b2_polygons;
}

void Box2DFactory::SetZero(Box2DLinkPtr const &link,
                           Eigen::Affine2d const &pose)
{
    link->set_pose(pose);

    BOOST_FOREACH (Box2DJointPtr const &joint, link->child_joints()) {
        Eigen::Affine2d const child_pose = pose * joint->origin();
        SetZero(joint->child_link(), child_pose);
    }
}

}
