#include <stdexcept>
#include <boost/assert.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

#include <geos/geom/Coordinate.h>
#include <geos/geom/Polygon.h>
#include <geos/geom/CoordinateSequence.h>
#include <geos/geom/CoordinateSequenceFactory.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/io/WKTReader.h>

#include "eigen_yaml.h"
#include "AffineTransformFilter.h"
#include "PolygonalLink.h"

geos::geom::Geometry *TransformGeometry(geos::geom::Geometry *input_geom,
                                        Eigen::Affine2d const &transform)
{
    AffineTransformFilter::Ptr filter = boost::make_shared<AffineTransformFilter>(transform);
    geos::geom::Geometry *output_geom = input_geom->clone();
    output_geom->apply_rw(filter.get());
    return output_geom;
}


/*
 * PolygonalLink
 */
PolygonalLink::PolygonalLink()
{
}

PolygonalLink::PolygonalLink(std::string const &name, Eigen::Affine2d const &relative_pose,
                             geos::geom::Geometry *geometry)
    : name_(name)
    , pose_(Eigen::Affine2d::Identity())
    , relative_pose_(relative_pose)
{
    BOOST_ASSERT(geometry);
    relative_geometry_.reset(TransformGeometry(geometry, relative_pose.inverse()));
}

PolygonalLink::~PolygonalLink()
{
}

void PolygonalLink::update()
{
    absolute_geometry_.reset();

    BOOST_FOREACH (PolygonalJoint::Ptr child_joint, joints_) {
        child_joint->update();
    }
}

std::string PolygonalLink::name() const
{
    return name_;
}

Eigen::Affine2d PolygonalLink::pose() const
{
    return pose_;
}

void PolygonalLink::set_pose(Eigen::Affine2d const &pose)
{
    pose_ = pose;
    relative_pose_ = pose;
    update();
}

Eigen::Affine2d PolygonalLink::relative_pose() const
{
    return relative_pose_;
}

boost::shared_ptr<geos::geom::Geometry const> PolygonalLink::geometry() const
{
    if (!absolute_geometry_) {
        absolute_geometry_.reset(TransformGeometry(relative_geometry_.get(), pose_));
    }
    return absolute_geometry_;
}

std::vector<PolygonalJoint::Ptr> PolygonalLink::joints() const
{
    return joints_;
}

std::vector<PolygonalLink::Ptr> PolygonalLink::children() const
{
    std::vector<PolygonalLink::Ptr> child_links;
    child_links.reserve(joints_.size());

    BOOST_FOREACH (PolygonalJoint::Ptr joint, joints_) {
        child_links.push_back(joint->child());
    }
    return child_links;
}

PolygonalJoint::Ptr PolygonalLink::AddJoint(std::string const &name, PolygonalLink::Ptr child_link,
                                            Eigen::Vector2d const &origin, int direction)
{
    PolygonalJoint::Ptr joint = boost::make_shared<PolygonalJoint>(
        name, shared_from_this(), child_link, origin, direction
    );
    joints_.push_back(joint);
    return joint;
}

void PolygonalLink::deserialize(YAML::Node const &node)
{
    geos::geom::GeometryFactory const *geom_factory = geos::geom::GeometryFactory::getDefaultInstance();
    geos::io::WKTReader geom_reader(geom_factory);
    std::string relative_geometry_wkt;

    name_ = node["name"].as<std::string>();

#ifdef YAMLCPP_NEWAPI
    relative_pose_ = node["relative_pose"].as<Eigen::Affine2d>();
#else
    node["relative_pose"] >> relative_pose_;
#endif

    relative_geometry_wkt = node["relative_geometry"].as<std::string>();
    relative_geometry_.reset(geom_reader.read(relative_geometry_wkt));
    BOOST_ASSERT(relative_geometry_);

    // Recursively deserialize the rest of the kinematic tree.
    joints_.clear();
    joints_.resize(node["joints"].size());
    for (size_t i = 0; i < node["joints"].size(); ++i) {
        PolygonalJoint::Ptr joint = boost::make_shared<PolygonalJoint>();
        joint->deserialize(shared_from_this(), node["joints"][i]);
        joints_[i] = joint;
    }

    pose_ = Eigen::Affine2d::Identity();
}

#ifdef YAMLCPP_NEWAPI

YAML::Node PolygonalLink::serialize() const
{
    YAML::Node node;
    node["name"] = name_;
    node["pose"] = pose_;
    node["relative_pose"] = relative_pose_;
    node["relative_geometry"] = relative_geometry_->toString();

    BOOST_FOREACH (PolygonalJoint::Ptr const &joint, joints_) {
        node["joints"].push_back(joint->serialize());
    }
}

#else

void PolygonalLink::serialize(YAML::Emitter &emitter) const
{
    emitter << YAML::BeginMap
            << YAML::Key << "name"              << YAML::Value << name_
            << YAML::Key << "pose"              << YAML::Value << pose_
            << YAML::Key << "relative_pose"     << YAML::Value << relative_pose_
            << YAML::Key << "relative_geometry" << YAML::Value << relative_geometry_->toString()
            << YAML::Key << "joints"            << YAML::Value << YAML::BeginSeq;

    BOOST_FOREACH (PolygonalJoint::Ptr joint, joints_) {
        emitter << *joint;
    }

    emitter << YAML::EndSeq
            << YAML::EndMap;
}

#endif

/*
 * PolygonalJoint
 */
PolygonalJoint::PolygonalJoint()
{
}

PolygonalJoint::PolygonalJoint(std::string const &name,
                               PolygonalLink::Ptr parent_link,
                               PolygonalLink::Ptr child_link,
                               Eigen::Vector2d const &origin, int direction)
    : name_(name)
    , parent_link_(parent_link)
    , child_link_(child_link)
    , direction_(direction)
    , angle_(0)
{
    BOOST_ASSERT(parent_link);
    BOOST_ASSERT(child_link);
    BOOST_ASSERT(parent_link != child_link);
    BOOST_ASSERT(direction == -1 || direction == 0 || direction == 1);

    relative_origin_ = Eigen::Affine2d::Identity();
    relative_origin_.rotate(Eigen::Rotation2Dd(-angle_));
    relative_origin_.pretranslate(origin);

    update();
}

std::string PolygonalJoint::name() const
{
    return name_;
}

Eigen::Affine2d PolygonalJoint::origin() const
{
    return parent_link_.lock()->pose() * relative_origin_;
}

PolygonalLink::Ptr PolygonalJoint::parent() const
{
    return parent_link_.lock();
}

PolygonalLink::Ptr PolygonalJoint::child() const
{
    return child_link_;
}

double PolygonalJoint::angle() const
{
    return angle_;
}

void PolygonalJoint::set_angle(double angle)
{
    angle_ = angle;
    update();
}

void PolygonalJoint::deserialize(PolygonalLink::Ptr parent_link, YAML::Node const &node)
{
    parent_link_ = parent_link;
    name_ = node["name"].as<std::string>();
    angle_ = node["angle"].as<double>();
    direction_ = node["direction"].as<double>();

#ifdef YAMLCPP_NEWAPI
    relative_origin_ = node["relative_origin"].as<Eigen::Affine2d>();
#else
    node["relative_origin"] >> relative_origin_;
#endif

    // Recursively deserialize the child link.
    child_link_ = boost::make_shared<PolygonalLink>();
    child_link_->deserialize(node["child_link"]);
}

#ifdef YAMLCPP_NEWAPI

YAML::Node PolygonalJoint::serialize() const
{
    YAML::Node node;
    node["name"] = name_;
    node["relative_origin"] = relative_origin_;
    node["direction"] = direction_ ;
    node["angle"] = angle_;
    node["child_link"] = child_link_->serialize();
    return node;
}

#else

void PolygonalJoint::serialize(YAML::Emitter &emitter) const
{
    emitter << YAML::BeginMap
            << YAML::Key << "name"            << YAML::Value << name_
            << YAML::Key << "relative_origin" << YAML::Value << relative_origin_
            << YAML::Key << "direction"       << YAML::Value << direction_ 
            << YAML::Key << "angle"           << YAML::Value << angle_
            << YAML::Key << "child_link"      << YAML::Value << *child_link_
            << YAML::EndMap;
}

#endif

void PolygonalJoint::update()
{
    // Update the pose of the child node.
    // TODO: Consider the joint angle.
    PolygonalLink::Ptr parent_link = parent_link_.lock();
    child_link_->pose_ = origin() * Eigen::Rotation2Dd(direction_ * angle_) * child_link_->relative_pose_;

    // Recursively update children.
    child_link_->update();
}

/*
 * Misc
 */
#ifndef YAMLCPP_NEWAPI

YAML::Emitter &operator<<(YAML::Emitter &emitter, PolygonalLink const &link)
{
    link.serialize(emitter);
    return emitter;
}

YAML::Emitter &operator<<(YAML::Emitter &emitter, PolygonalJoint const &joint)
{
    joint.serialize(emitter);
    return emitter;
}

#endif
