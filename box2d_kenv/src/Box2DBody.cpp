#include <list>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include "Box2DBody.h"
#include "Box2DLink.h"
#include "Box2DJoint.h"
#include "Box2DWorld.h"

using boost::format;
using boost::str;

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

std::string Box2DBody::name() const
{
    return name_;
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

std::vector<Box2DLinkPtr> Box2DBody::links() const
{
    CheckInitialized();

    std::vector<Box2DLinkPtr> links;
    GetChildren(root_link_, &links, NULL);

    return links;
}

std::vector<Box2DJointPtr> Box2DBody::joints() const
{
    CheckInitialized();

    std::vector<Box2DJointPtr> joints;
    GetChildren(root_link_, NULL, &joints);

    return joints;
}

void Box2DBody::Initialize(Box2DLinkPtr const &root_link)
{
    BOOST_ASSERT(root_link);

    if (root_link_) {
        throw std::runtime_error(
            str(format("Body '%s' has already been initialized") % name_)
        );
    } else if (root_link->parent_body().get() != this) {
        throw std::runtime_error(
            str(format("Link has incorrect body: expected '%s', got '%s'.")
                % name_ % root_link->parent_body()->name())
        );
    }

    // Generate a list of all links and joints attached to this body.:w
    links_.clear();
    joints_.clear();
    GetChildren(root_link, &links_, &joints_);

    // Set the links' initial poses. Otherwise, the links could overlap and
    // generate a large transient force.
    SetZero(root_link, Eigen::Affine2d::Identity());

    root_link_ = root_link;
}

void Box2DBody::CheckInitialized() const
{
    if (!root_link_) {
        throw std::runtime_error(
            str(format("Body '%s' has not been fully constructed.") % name_));
    }
}

void Box2DBody::GetChildren(Box2DLinkPtr const &root_link,
                            std::vector<Box2DLinkPtr> *links,
                            std::vector<Box2DJointPtr> *joints) const
{
    std::list<Box2DLinkPtr> pending_links;
    pending_links.push_front(root_link);

    while (!pending_links.empty()) {
        Box2DLinkPtr const &link = pending_links.front();

        if (links != NULL) {
            links->push_back(link);
        }

        BOOST_FOREACH (Box2DJointPtr const &joint, link->child_joints()) {
            if (joints != NULL) {
                joints->push_back(joint);
            }

            GetChildren(joint->child_link(), links, joints);
        }

        pending_links.pop_front();
    }
}


void Box2DBody::SetZero(Box2DLinkPtr const &link,
                        Eigen::Affine2d const &pose)
{
    link->set_pose(pose);

    BOOST_FOREACH (Box2DJointPtr const &joint, link->child_joints()) {
        Eigen::Affine2d const child_pose = pose * joint->origin();
        SetZero(joint->child_link(), child_pose);
    }
}

}
