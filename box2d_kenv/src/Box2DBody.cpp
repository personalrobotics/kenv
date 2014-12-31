#include <boost/format.hpp>
#include "Box2DBody.h"
#include "Box2DLink.h"
#include "Box2DWorld.h"

namespace box2d_kenv {

Box2DBody::Box2DBody(Box2DWorldPtr const &world, std::string const &name)
    : name_(name)
    , world_(world)
{
    BOOST_ASSERT(world);
}

Box2DWorldPtr Box2DBody::world() const
{
    return world_.lock();
}

Box2DLinkPtr Box2DBody::root_link()
{
    return root_link_;
}

Eigen::Affine2d Box2DBody::pose() const
{
    CheckInitialized();

    return root_link_->pose();
}

void Box2DBody::set_pose(Eigen::Affine2d const &pose)
{
    CheckInitialized();

    return root_link_->set_pose(pose);
}

Eigen::Vector3d Box2DBody::twist() const
{
    CheckInitialized();

    return root_link_->twist();
}

void Box2DBody::set_twist(Eigen::Vector3d const &twist)
{
    CheckInitialized();

    return root_link_->set_twist(twist);
}

void Box2DBody::CheckInitialized() const
{
    using boost::format;
    using boost::str;

    if (!root_link_) {
        throw std::runtime_error(
            str(format("Body '%s' has not been fully constructed.") % name_));
    }
}

}
