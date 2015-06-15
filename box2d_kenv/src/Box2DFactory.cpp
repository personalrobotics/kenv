#include <stdexcept>
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
#include "Box2DBody.h"
#include "Box2DFactory.h"
#include "Box2DJoint.h"
#include "Box2DLink.h"
#include "Box2DSensor.h"
#include "Box2DWorld.h"
#include "yaml_utils.h"

using boost::format;
using boost::make_shared;
using boost::str;

typedef boost::shared_ptr<geos::geom::Geometry> GeometryPtr;

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
    Box2DLinkPtr const root_link = CreateLink(body, node);
    body->Initialize(root_link);

    return body;
}

Box2DBodyPtr Box2DFactory::CreateEmptyBody(std::string const &name)
{
    b2BodyDef b2_bodydef;
    b2_bodydef.type = b2_staticBody;
    b2Body *b2_body = b2_world_->CreateBody(&b2_bodydef);

    GeometryPtr const empty_geometry(
        geos::geom::GeometryFactory::getDefaultInstance()
            ->createEmptyGeometry()
    );

    Box2DBodyPtr const body = make_shared<Box2DBody>(world_, name);
    Box2DLinkPtr const link
        = make_shared<Box2DLink>(body, "empty", empty_geometry, b2_body);
    body->Initialize(link);

    return body;
}

std::vector<Box2DSensorPtr> Box2DFactory::CreateSensors(
        Box2DBodyPtr const &body, YAML::Node const &node)
{
    static Eigen::Affine2d const identity = Eigen::Affine2d::Identity();
    double const scale = world_->scale();

    std::vector<Box2DSensorPtr> all_sensors;

    for (size_t i = 0; i < node.size(); ++i) {
        YAML::Node const &sensor_node = node[i];

#if YAMLCPP_NEWAPI
        std::string const name = sensor_node["name"].as<std::string>();
        std::string const geometry_wkt
            = sensor_node["geometry"].as<std::string>();
        std::string const parent_link_name
            = sensor_node["parent_link"].as<std::string>();
#else
        std::string name, geometry_wkt, parent_link_name;
        sensor_node["name"] >> name;
        sensor_node["geometry"] >> geometry_wkt;
        sensor_node["parent_link"] >> parent_link_name;
#endif
        
        Box2DLinkPtr const parent_link = body->GetLink(parent_link_name);
        b2Body *const b2_body = parent_link->b2_body();

        // Load the sensor geometry. If the sensor is a LineString, convert it
        // to a Polygon that will be accepted by Box2D.
        GeometryPtr const geometry(wkt_reader_.read(geometry_wkt));
        GeometryPtr modified_geometry = geometry;
        if (geometry->getGeometryTypeId() == geos::geom::GEOS_LINESTRING) {
            // TODO: In theory, (2 * b2_linearSlop + epsilon) should work. Why
            // do I need such a large value here?
            modified_geometry.reset(
                geometry->buffer(100. * b2_linearSlop / scale, 1)
            );
        }
        if (modified_geometry->getGeometryTypeId() != geos::geom::GEOS_POLYGON) {
            throw std::runtime_error(boost::str(
                boost::format("Sensor for link '%s' has type '%s'; only polygons"
                              " are supported.")
                    % parent_link_name % geometry->getGeometryType()
            ));
        }
        geos::geom::Polygon const &polygon
            = dynamic_cast<geos::geom::Polygon const &>(*modified_geometry);
        std::vector<b2PolygonShape> const b2_shapes
            = ConvertGeometry(polygon, identity);

        // Create mass-less fixtures for the sensor. These fixtures are flagged
        // with the isSensor flag, so they will not impact the physics simulation.
        std::vector<b2Fixture *> b2_fixtures;

        BOOST_FOREACH (b2PolygonShape const &b2_shape, b2_shapes) {
            b2FixtureDef b2_fixturedef;
            b2_fixturedef.shape = &b2_shape;
            b2_fixturedef.density = 0.;
            b2_fixturedef.isSensor = true;

            b2Fixture *const b2_fixture = b2_body->CreateFixture(&b2_fixturedef);
            b2_fixtures.push_back(b2_fixture);
        }

        // Create the wrapper object.
        Box2DSensorPtr const sensor = boost::make_shared<Box2DSensor>(
            parent_link, name, geometry, b2_fixtures);
        parent_link->AddSensor(sensor);
        all_sensors.push_back(sensor);
    }

    return all_sensors;
}

Box2DLinkPtr Box2DFactory::CreateLink(Box2DBodyPtr const &parent_body,
                                      YAML::Node const &node)
{
    using namespace box2d_kenv::util;

    BOOST_ASSERT(parent_body);

    std::string name;
    std::string geometry_wkt;
    Eigen::Affine2d relative_pose;
    double density, friction, restitution;
    double const scale = parent_body->world()->scale();

    try {
#ifdef YAMLCPP_NEWAPI
        name = node["name"].as<std::string>();
        geometry_wkt = node["relative_geometry"].as<std::string>();
        relative_pose = node["relative_pose"].as<Eigen::Affine2d>();
        density = node["density"].as<double>();
        friction = node["friction"].as<double>();
        restitution = node["restitution"].as<double>();
#else
        node["name"] >> name;
        node["relative_geometry"] >> geometry_wkt;
        node["relative_pose"] >> relative_pose;
        node["density"] >> density;
        node["friction"] >> friction;
        node["restitution"] >> restitution;
#endif
    } catch (YAML::Exception const &e) {
        throw std::runtime_error(
            str(format("Failed loading link '%s': %s") % name % e.what())
        );
    }

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
    //b2_bodydef.allowSleep = true;
    b2_bodydef.allowSleep = false;
    b2_bodydef.type = b2_dynamicBody;

    b2Body *b2_body = b2_world_->CreateBody(&b2_bodydef);
    std::vector<b2PolygonShape> const b2_shapes
        = ConvertGeometry(polygon, relative_pose);
    std::vector<b2Fixture *> b2_fixtures;

    BOOST_FOREACH (b2PolygonShape const &b2_shape, b2_shapes) {
        // TODO: Load these properties from YAML.
        b2FixtureDef b2_fixturedef;
        b2_fixturedef.shape = &b2_shape;
        b2_fixturedef.density = density;

        // NOTE: Box2D computes the effective coefficient of friction as the
        // geometric mean of the coefficients of friction of the two contacting
        // bodies. It's not clear if this is desirable.
        b2_fixturedef.friction = friction;

        // NOTE: Box2D computes the effective restitution value as the max of
        // the restitution values of the two contacting bodies. It's not clear
        // if this is desirable.
        b2_fixturedef.restitution = restitution;

        b2Fixture *const b2_fixture = b2_body->CreateFixture(&b2_fixturedef);
        b2_fixtures.push_back(b2_fixture);
    }

    // Create this link.
    Box2DLinkPtr const link
        = make_shared<Box2DLink>(parent_body, name, geometry, b2_body);

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
    std::string name;
    Eigen::Affine2d relative_origin;
    double direction;

    try {
#ifdef YAMLCPP_NEWAPI
        name = node["name"].as<std::string>();
        relative_origin = node["relative_origin"].as<Eigen::Affine2d>();
        direction = node["direction"].as<double>();
#else
        node["name"] >> name;
        node["relative_origin"] >> relative_origin;
        node["direction"] >> direction;
#endif
    } catch (YAML::Exception const &e) {
        throw std::runtime_error(
            str(format("Failed loading joint '%s' <-> '%s': %s")
                % parent_link->name() % child_link->name() % e.what())
        );
    }

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
    
    if (direction == 0.) {
        b2_jointdef.enableLimit = true;
        b2_jointdef.lowerAngle = 0.;
        b2_jointdef.upperAngle = 0.;
    } else {
        // TODO: Read joint limits from YAML.
        b2_jointdef.enableLimit = false;
    }

    // TODO: Read the maximum motor torque from YAML.
    b2_jointdef.enableMotor = true;
    b2_jointdef.maxMotorTorque = 1e8;

    b2RevoluteJoint *b2_joint = static_cast<b2RevoluteJoint *>(
        b2_world_->CreateJoint(&b2_jointdef));

    return make_shared<Box2DJoint>(name, parent_link, child_link,
                                   b2_joint, direction);
}

std::vector<b2PolygonShape> Box2DFactory::ConvertGeometry(
        geos::geom::Polygon const &geom, Eigen::Affine2d const &transform)
{
    using namespace ::box2d_kenv::util;

    typedef boost::shared_ptr<geos::geom::Polygon> PolygonPtr;

    double const scale = world_->scale();
    double const weld_distance = 0.5 * b2_linearSlop / scale;

    // Transform the geometry into the link frame.
    PolygonPtr geom_clone(dynamic_cast<geos::geom::Polygon *>(geom.clone()));
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

}
