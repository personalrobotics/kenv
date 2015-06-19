#include <stdexcept>
#include "Box2DLink.h"
#include "Box2DJoint.h"
#include "Box2DWorld.h"

namespace box2d_kenv {

Box2DJoint::Box2DJoint(std::string const &name,
                       Box2DLinkPtr parent_link,
                       Box2DLinkPtr child_link,
                       b2RevoluteJoint *b2_joint,
                       double direction)
    : name_(name)
    , parent_link_(parent_link)
    , child_link_(child_link)
    , b2_joint_(b2_joint)
    , direction_(direction)
{
    BOOST_ASSERT(parent_link);
    BOOST_ASSERT(child_link);
    BOOST_ASSERT(parent_link != child_link);
    BOOST_ASSERT(b2_joint);
    BOOST_ASSERT(direction == -1. || direction == 0. || direction == 1.);
}

Box2DJoint::~Box2DJoint()
{
    b2_joint_->GetBodyA()->GetWorld()->DestroyJoint(b2_joint_);
}

Eigen::Affine2d Box2DJoint::origin() const
{
    b2Vec2 const anchor = b2_joint_->GetLocalAnchorA();
    double const scale = parent_link()->world()->scale();

    Eigen::Affine2d origin = Eigen::Affine2d::Identity();
    origin.rotate(Eigen::Rotation2Dd(b2_joint_->GetReferenceAngle()));
    origin.pretranslate(Eigen::Vector2d(anchor.x / scale, anchor.y / scale));

    return origin;
}

std::string Box2DJoint::name() const
{
    return name_;
}

Box2DLinkPtr Box2DJoint::parent_link() const
{
    return parent_link_.lock();
}

Box2DLinkPtr Box2DJoint::child_link() const
{
    return child_link_;
}

b2RevoluteJoint *Box2DJoint::b2_joint()
{
    return b2_joint_;
}

b2RevoluteJoint const *Box2DJoint::b2_joint() const
{
    return b2_joint_;
}

double Box2DJoint::value() const
{
    return direction_ * b2_joint_->GetJointAngle();
}

void Box2DJoint::set_value(double value)
{
    b2Vec2 const anchor = b2_joint_->GetLocalAnchorA();
    double const scale = parent_link()->world()->scale();

    Eigen::Affine2d relative = Eigen::Affine2d::Identity();
    relative.rotate(Eigen::Rotation2Dd(b2_joint_->GetReferenceAngle()
                                       + direction_ * value));
    relative.pretranslate(Eigen::Vector2d(anchor.x / scale, anchor.y / scale));

    Eigen::Affine2d const child_pose = parent_link()->pose() * relative;
    child_link()->set_pose(child_pose);
}

double Box2DJoint::velocity() const
{
    return direction_ * b2_joint_->GetJointSpeed();
}

#if 0
void Box2DJoint::set_velocity(double velocity)
{
    // Get the twist of the parent frame.
    Eigen::Vector3d const parent_twist = parent_link()->twist();

    // TODO: Compute the velocity induced by the rotation of this joint.
    // TODO: Set the velocitiy of the child links.
}
#endif

void Box2DJoint::set_desired_velocity(double velocity)
{
    if (direction_ == 0.) {
        throw std::runtime_error("Unable to set velocity on fixed joint.");
    }

    b2_joint_->SetMotorSpeed(direction_ * velocity);
}

}