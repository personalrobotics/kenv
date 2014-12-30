#include <stdexcept>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include "Box2DBody.h"
#include "Box2DLink.h"
#include "Box2DJoint.h"
#include "Box2DWorld.h"

namespace box2d_kenv {

Box2DLink::Box2DLink(Box2DBodyPtr body, std::string const &name,
                     b2Body *b2_body)
    : parent_body_(body)
    , name_(name)
    , b2_body_(b2_body)
{
    BOOST_ASSERT(body);
    BOOST_ASSERT(b2_body_);
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

b2Body *Box2DLink::b2_body() {
    return b2_body_;
}

b2Body const *Box2DLink::b2_body() const
{
    return b2_body_;
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

}
