#include <stdexcept>
#include "Box2DJoint.h"

namespace box2d_kenv {

Box2DJoint::Box2DJoint(Box2DLinkPtr parent_link,
                       Box2DLinkPtr child_link,
                       b2RevoluteJoint *b2_joint)
    : parent_link_(parent_link)
    , child_link_(child_link)
    , b2_joint_(b2_joint)
{
    BOOST_ASSERT(parent_link);
    BOOST_ASSERT(child_link);
    BOOST_ASSERT(parent_link != child_link);
    BOOST_ASSERT(b2_joint);
}

Eigen::Affine2d Box2DJoint::origin() const
{
    b2Vec2 const anchor = b2_joint_->GetLocalAnchorA();

    Eigen::Affine2d origin = Eigen::Affine2d::Identity();
    origin.rotate(Eigen::Rotation2Dd(b2_joint_->GetReferenceAngle()));
    origin.pretranslate(Eigen::Vector2d(anchor.x, anchor.y));

    return origin;
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
    return b2_joint_->GetJointAngle();
}

void Box2DJoint::set_value(double value)
{
    throw std::runtime_error("not implemented");
}

}
