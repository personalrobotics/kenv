#include <fstream>
#include <list>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/typeof/typeof.hpp>
#include <Box2D/Dynamics/Joints/b2FrictionJoint.h>
#include "Box2DBody.h"
#include "Box2DLink.h"
#include "Box2DJoint.h"
#include "Box2DWorld.h"

using boost::format;
using boost::str;

namespace {

template <typename T>
std::vector<boost::shared_ptr<T const> > ToConst(
    std::vector<boost::shared_ptr<T> > const &v)
{
    std::vector<boost::shared_ptr<T const> > v_const;
    v_const.resize(v.size());
    
    for (size_t i = 0; i < v.size(); ++i) {
        v_const[i] = v[i];
    }

    return v_const;
}

}

namespace box2d_kenv {

Box2DBody::Box2DBody(Box2DWorldPtr const &world, std::string const &name)
    : name_(name)
    , world_(world)
{
    BOOST_ASSERT(world);
}

Box2DBody::~Box2DBody()
{
    joints_.clear();
    joints_map_.clear();

    root_link_.reset();
    links_.clear();
    links_map_.clear();
    sensors_.clear();
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

    root_link_->set_pose(pose);
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

Box2DLinkPtr Box2DBody::GetLink(std::string const &name)
{
    BOOST_AUTO(it, links_map_.find(name));
    if (it != links_map_.end()) {
        return it->second;
    } else {
        throw std::runtime_error(
            str(format("There is no link named '%s'.") % name));
    }
}

Box2DLinkConstPtr Box2DBody::GetLink(std::string const &name) const
{
    BOOST_AUTO(it, links_map_.find(name));
    if (it != links_map_.end()) {
        return it->second;
    } else {
        throw std::runtime_error(
            str(format("There is no link named '%s'.") % name));
    }
}

Box2DJointPtr Box2DBody::GetJoint(std::string const &name)
{
    BOOST_AUTO(it, joints_map_.find(name));
    if (it != joints_map_.end()) {
        return it->second;
    } else {
        throw std::runtime_error(
            str(format("There is no joint named '%s'.") % name));
    }
}

Box2DJointConstPtr Box2DBody::GetJoint(std::string const &name) const
{
    BOOST_AUTO(it, joints_map_.find(name));
    if (it != joints_map_.end()) {
        return it->second;
    } else {
        throw std::runtime_error(
            str(format("There is no joint named '%s'.") % name));
    }
}

std::vector<Box2DLinkPtr> Box2DBody::links()
{
    CheckInitialized();

    return links_;
}

std::vector<Box2DLinkConstPtr> Box2DBody::links() const
{
    CheckInitialized();

    return ToConst(links_);
}

std::vector<Box2DJointPtr> Box2DBody::joints()
{
    CheckInitialized();

    return joints_;
}

std::vector<Box2DJointConstPtr> Box2DBody::joints() const
{
    CheckInitialized();

    return ToConst(joints_);
}

std::vector<Box2DSensorPtr> Box2DBody::sensors()
{
    CheckInitialized();

    return sensors_;
}

std::vector<Box2DSensorConstPtr> Box2DBody::sensors() const
{
    CheckInitialized();

    return ToConst(sensors_);
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
    sensors_.clear();
    links_map_.clear();
    joints_map_.clear();
    GetChildren(root_link);

    // Set the links' initial poses. Otherwise, the links could overlap and
    // generate a large transient force.
    SetZero(root_link, Eigen::Affine2d::Identity());

    root_link_ = root_link;
}


void Box2DBody::CreateSensors(std::istream &stream)
{
#ifdef YAMLCPP_NEWAPI
    YAML::Node node = YAML::Load(stream);
#else
    YAML::Parser parser(stream);
    YAML::Node node;
    parser.GetNextDocument(node);
#endif

    Box2DFactory factory(world());
    std::vector<Box2DSensorPtr> const sensors
        = factory.CreateSensors(shared_from_this(), node);
    sensors_.insert(sensors_.end(), sensors.begin(), sensors.end());
}

void Box2DBody::CreateSensors(std::string const &path)
{
    std::ifstream stream(path.c_str());

    if (!stream.good()) {
        throw std::runtime_error(
            str(format("Unable to read sensors for body '%s' from '%s'.")
                % name_ % path));
    }
    return CreateSensors(stream);
}


/*
 * Private
 */
void Box2DBody::CheckInitialized() const
{
    if (!root_link_) {
        throw std::runtime_error(
            str(format("Body '%s' has not been fully constructed.") % name_));
    }
}

void Box2DBody::GetChildren(Box2DLinkPtr const &root_link)
{
    std::list<Box2DLinkPtr> pending_links;
    pending_links.push_front(root_link);

    while (!pending_links.empty()) {
        Box2DLinkPtr const &link = pending_links.front();

        // Build the list of links.
        links_.push_back(link);
        BOOST_AUTO(link_result, links_map_.insert(
            std::make_pair(link->name(), link)));
        if (!link_result.second) {
            throw std::runtime_error(
                str(format("Duplicate link named '%s'.") % link->name()));
        }

        BOOST_FOREACH (Box2DJointPtr const &joint, link->child_joints()) {
            // Build the list of joints.
            joints_.push_back(joint);
            BOOST_AUTO(joint_result, joints_map_.insert(
                std::make_pair(joint->name(), joint)));
            if (!joint_result.second) {
                throw std::runtime_error(
                    str(format("Duplicate joint named '%s'.") % joint->name()));
            }

            // Recursively process the remainder of the links in this subtree.
            GetChildren(joint->child_link());
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
