#include <stdexcept>
#include <boost/format.hpp>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/Joints/b2FrictionJoint.h>
#include "Box2DBody.h"
#include "Box2DJoint.h"
#include "Box2DLink.h"
#include "Box2DSensor.h"
#include "Box2DWorld.h"

using boost::format;
using boost::str;

namespace box2d_kenv {

Box2DLink::Box2DLink(Box2DBodyPtr body,
                     std::string const &name,
                     GeometryConstPtr const &geometry,
                     b2Body *b2_body)
    : parent_body_(body)
    , name_(name)
    , b2_body_(b2_body)
    , geometry_(geometry)
    , b2_friction_(NULL)
{
    BOOST_ASSERT(body);
    BOOST_ASSERT(b2_body_);

    // Setup back-references from Box2D.
    b2_body->SetUserData(this);

    for (b2Fixture *b2_fixture = b2_body->GetFixtureList();
         b2_fixture != NULL;
         b2_fixture = b2_fixture->GetNext())
    {
        b2_fixture->SetUserData(this);
    }
}

Box2DWorldPtr Box2DLink::world() const
{
    return parent_body_.lock()->world();
}

Box2DBodyPtr Box2DLink::parent_body() const
{
    return parent_body_.lock();
}

Box2DLinkPtr Box2DLink::parent_link() const
{
    return parent_joint_.lock()->parent_link();
}

Box2DJointPtr Box2DLink::parent_joint() const
{
    return parent_joint_.lock();
}

std::vector<Box2DJointPtr> Box2DLink::child_joints() const
{
    return std::vector<Box2DJointPtr>(child_joints_.begin(),
                                      child_joints_.end());
}

std::vector<Box2DSensorPtr> Box2DLink::child_sensors() const
{
    return std::vector<Box2DSensorPtr>(child_sensors_.begin(),
                                       child_sensors_.end());
}

b2Body *Box2DLink::b2_body() {
    return b2_body_;
}

b2Body const *Box2DLink::b2_body() const
{
    return b2_body_;
}

std::string Box2DLink::name() const
{
    return name_;
}

GeometryConstPtr Box2DLink::geometry() const
{
    return geometry_;
}

std::vector<b2Fixture const *> Box2DLink::fixtures() const
{
    std::vector<b2Fixture const *> fixtures;

    for (b2Fixture const *fixture = b2_body_->GetFixtureList();
         fixture != NULL;
         fixture = fixture->GetNext()) {
        fixtures.push_back(fixture);
    }

    return fixtures;
}

Eigen::Affine2d Box2DLink::pose() const
{
    double const scale = world()->scale();
    b2Vec2 const &position = b2_body_->GetPosition();
    float32 const angle = b2_body_->GetAngle();

    Eigen::Affine2d pose;
    pose.rotate(Eigen::Rotation2Dd(angle));
    pose.translation()[0] = scale * position.x;
    pose.translation()[1] = scale * position.y;

    return pose;
}

void Box2DLink::set_pose(Eigen::Affine2d const &pose)
{
    double const scale = world()->scale();

    Eigen::Rotation2Dd rotation(0.);
    rotation.fromRotationMatrix(pose.rotation());

    b2_body_->SetTransform(
        b2Vec2(scale * pose.translation()[0],
               scale * pose.translation()[1]),
        rotation.angle()
    );
}

Eigen::Vector3d Box2DLink::twist() const
{
    double const scale = world()->scale();

    b2Vec2 const &linear_velocity = b2_body_->GetLinearVelocity();
    float32 const angular_velocity = b2_body_->GetAngularVelocity();

    return Eigen::Vector3d(
        scale * linear_velocity.x,
        scale * linear_velocity.y,
        angular_velocity
    );
}

void Box2DLink::set_twist(Eigen::Vector3d const &twist) const
{
    double const scale = world()->scale();

    b2_body_->SetLinearVelocity(b2Vec2(scale * twist[0],
                                       scale * twist[1]));
    b2_body_->SetAngularVelocity(twist[2]);
}

double Box2DLink::mass() const
{
    return b2_body_->GetMass();
}

double Box2DLink::rotational_inertia() const
{
    return b2_body_->GetInertia();
}

void Box2DLink::set_inertia(double mass, double rotational_inertia)
{
    b2MassData b2_mass;
    b2_body_->GetMassData(&b2_mass);

    b2_mass.mass = mass;
    b2_mass.I = rotational_inertia;

    b2_body_->SetMassData(&b2_mass);
}

void Box2DLink::AddChildJoint(Box2DJointPtr const &joint)
{
    BOOST_ASSERT(joint);

    if (joint->parent_link().get() != this) {
        throw std::runtime_error("Joint does not have this link as its parent.");
    } else if (joint->child_link()->parent_joint()) {
        throw std::runtime_error("Child link already has a parent joint.");
    }

    joint->child_link()->parent_joint_ = joint;
    std::pair<std::set<Box2DJointPtr>::iterator, bool> const result
        = child_joints_.insert(joint);

    if (!result.second) {
        throw std::runtime_error("Attempted to add a duplicate joint.");
    }
}

void Box2DLink::AddSensor(Box2DSensorPtr const &sensor)
{
    if (sensor->parent_link().get() != this) {
        throw std::runtime_error("Sensor does not have thislink as its parent.");
    }

    std::pair<std::set<Box2DSensorPtr>::iterator, bool> const result
        = child_sensors_.insert(sensor);

    if (!result.second) {
        throw std::runtime_error("Attempted to add a duplicate sensor.");
    }
}

#if 0
    // Create a Box2D sensor fixture. This fixture will generate contact
    // events, but will not influence the physics simulation.
    // TODO: Should I setup the contact filter?
    b2FixtureDef b2_fixturedef;
    b2_fixturedef.shape = geometry;
    b2_fixturedef.density = 0.;
    b2_fixturedef.friction = 0.;
    b2_fixturedef.restitution = 0.;
    b2_fixturedef.isSensor = true;
    b2Fixture *fixture = b2_body_->CreateFixture(&b2_fixturedef);

    // Create the wrapper object.
    Box2DSensorPtr const sensor = boost::make_shared<Box2DSensor>(
        shared_from_this(), fixture);
    child_sensors_.insert(sensor);
    return sensor;
#endif

#if 0
double Box2DLink::friction_coefficient() const
{
    if (!b2_friction_) {
        throw std::runtime_error(
            str(format("Body '%s' does not have friction enabled.") % name_)
        );
    }

    double const scale = world()->scale();
    double const mass = 

    return b2_friction_->GetMaxForce() / (scale * b2_body->GetMass());
}

double Box2DLink::pressure_radius() const
{
    if (!b2_friction_) {
        throw std::runtime_error(
            str(format("Body '%s' does not have friction enabled.") % name_)
        );
    }

    double const scale = world()->scale();
    double const mu = friction_coefficient();

    return b2_friction_->GetMaxTorque() / (scale * mu * b2_body->GetMass());
}
#endif

void Box2DLink::enable_friction(Box2DLinkPtr const &surface)
{
    if (b2_friction_) {
        throw std::runtime_error(
            str(format("Body '%s' already has friction enabled with '%s'.")
                % name_ % surface->name())
        );
    }

    b2FrictionJointDef b2_jointdef;
    b2_jointdef.Initialize(b2_body_, surface->b2_body_, b2Vec2(0., 0.));
    b2_jointdef.maxForce = 0.;
    b2_jointdef.maxTorque = 0.;

    b2_friction_ = static_cast<b2FrictionJoint *>(
        world()->b2_world()->CreateJoint(&b2_jointdef));
}

void Box2DLink::set_friction(double mu, double c)
{
    static double const gravity = 9.81;

    if (!b2_friction_) {
        throw std::runtime_error(
            str(format("Body '%s' does not have friction enabled.") % name_)
        );
    }

    double const scale = world()->scale();
    double const max_normal_force = scale * gravity * b2_body_->GetMass();

    b2_friction_->SetMaxForce(mu * max_normal_force);
    b2_friction_->SetMaxTorque(mu * c * max_normal_force);
}

}
