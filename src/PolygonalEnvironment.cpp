#include <fstream>
#include <boost/assert.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/typeof/typeof.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/make_shared.hpp>
#include <geos/geom/Geometry.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/LineString.h>
#include <geos/geom/MultiLineString.h>
#include <geos/geom/Polygon.h>
#include <geos/operation/union/CascadedPolygonUnion.h>
#include "PolygonalEnvironment.h"

using namespace boost::assign;

typedef boost::shared_ptr<void> Handle;

inline Eigen::Vector3d toEigen(geos::geom::Coordinate const &coord)
{
    return Eigen::Vector3d(coord.x, coord.y, 0.0);
}

inline Eigen::Vector3d orthgonal(Eigen::Vector3d const &x)
{
    BOOST_ASSERT(x[2] == 0.0);
    return Eigen::Vector3d(-x[1], x[0], 0.0);
}


namespace kenv {

/*
 * PolygonalLink
 */
PolygonalLink::PolygonalLink(::PolygonalLink::Ptr internal_link)
    : enable_(true)
    , internal_link_(internal_link)
{
    BOOST_ASSERT(internal_link_);
}

boost::shared_ptr<Object> PolygonalLink::getObject(void) const
{
    throw std::runtime_error("not implemented");
}

std::string PolygonalLink::getName(void) const
{
    return internal_link_->name();
}

geos::geom::Geometry *PolygonalLink::getGeometry() const
{
    return internal_link_->geometry();
}

std::vector<geos::geom::Geometry *> PolygonalLink::getSensors() const
{
    return internal_link_->sensors();
}

Eigen::Affine3d PolygonalLink::getTransform(void) const
{
    Eigen::Affine2d const pose_2d = internal_link_->pose();
    Eigen::Affine3d pose_3d = Eigen::Affine3d::Identity();
    pose_3d.matrix().block<2, 2>(0, 0) = pose_2d.matrix().block<2, 2>(0, 0);
    pose_3d.translation().head<2>() = pose_2d.translation();
    return pose_3d;
}

void PolygonalLink::enable(bool flag)
{
    enable_ = flag;
}

AlignedBox3d PolygonalLink::computeLocalAABB()
{
    throw std::runtime_error("not implemented");
}

/*
 * PolygonalObject
 */
PolygonalObject::PolygonalObject(std::string const &type, std::string const &name,
                                 boost::shared_ptr<PolygonalEnvironment> environment,
                                 ::PolygonalLink::Ptr base_link)
    : type_(type)
    , name_(name)
    , transparency_(0)
    , color_(1, 0, 0, 1)
    , environment_(environment)
    , base_link_(base_link)
{
    BOOST_ASSERT(!type.empty());
    BOOST_ASSERT(!name.empty());
    BOOST_ASSERT(environment);
    BOOST_ASSERT(base_link);

    // Build the vector of joints and joints.
    std::vector< ::PolygonalLink::Ptr> queue;
    queue.push_back(base_link_);

    while (!queue.empty()) {
        ::PolygonalLink::Ptr raw_link = queue.back();
        queue.pop_back();

        PolygonalLink::Ptr link = boost::make_shared<PolygonalLink>(raw_link);
        links_.push_back(link);

        BOOST_FOREACH (::PolygonalJoint::Ptr joint, raw_link->joints()) {
            joints_.push_back(joint);
            queue.push_back(joint->child());
        }
    }
}

boost::shared_ptr<Environment> PolygonalObject::getEnvironment(void) const
{
    return environment_.lock();
}

std::string PolygonalObject::getName(void) const
{
    return name_;
}

std::string PolygonalObject::getType(void) const
{
    return type_;
}

std::string PolygonalObject::getKinematicsGeometryHash(void) const
{
    throw std::runtime_error("not implemented");
}

void PolygonalObject::enable(bool flag)
{
    BOOST_FOREACH (Link::Ptr link, getLinks()) {
        link->enable(flag);
    }
}

void PolygonalObject::setVisible(bool flag)
{
    throw std::runtime_error("not implemented");
}

AlignedBox3d PolygonalObject::getAABB(void) const
{
    throw std::runtime_error("not implemented");
}


bool PolygonalObject::checkCollision(Object::ConstPtr entity, std::vector<Contact> *contacts,
                                     std::vector<std::pair<Link::Ptr, Link::Ptr> > *links) const
{
    static double const resolution = 0.0025;

    PolygonalObject::ConstPtr other = boost::dynamic_pointer_cast<PolygonalObject const>(entity);
    BOOST_ASSERT(other);

    geos::geom::Geometry *geom1 = getGeometry();
    geos::geom::Geometry *geom2 = other->getGeometry();

    if (links) {
        links->clear();

        std::vector<kenv::Link::Ptr> links1 = getLinks();
        std::vector<kenv::Link::Ptr> links2 = other->getLinks();

        BOOST_FOREACH (kenv::Link::Ptr link1, links1)
        BOOST_FOREACH (kenv::Link::Ptr link2, links2) {
            PolygonalLink::Ptr polygonal_link1 = boost::dynamic_pointer_cast<PolygonalLink>(link1);
            PolygonalLink::Ptr polygonal_link2 = boost::dynamic_pointer_cast<PolygonalLink>(link2);
            geos::geom::Geometry *geom_link1 = polygonal_link1->getGeometry();
            geos::geom::Geometry *geom_link2 = polygonal_link2->getGeometry();

            if (geom_link1->intersects(geom_link2)) {
                links->push_back(std::make_pair(link1, link2));
            }
        }
    }

    if (contacts) {
        std::vector<geos::geom::LineString const *> linestrings;

        // Compute the intersection line(s).
        geos::geom::Geometry *boundary = geom1->getBoundary()->intersection(geom2);
        if (boundary->isEmpty()) {
            return false;
        }

        std::vector<geos::geom::LineString *> lines;
        switch (boundary->getGeometryTypeId()) {
        case geos::geom::GEOS_LINESTRING: {
            geos::geom::LineString *linestring = dynamic_cast<geos::geom::LineString *>(boundary);
            BOOST_ASSERT(linestring);
            linestrings.push_back(linestring);
            break;
        }

        case geos::geom::GEOS_MULTILINESTRING: {
            geos::geom::MultiLineString *multilinestring = dynamic_cast<geos::geom::MultiLineString *>(boundary);
            BOOST_ASSERT(multilinestring);

            for (size_t i = 0; i < multilinestring->getNumGeometries(); ++i) {
                geos::geom::Geometry const *tmp = multilinestring->getGeometryN(i);
                geos::geom::LineString const *linestring = dynamic_cast<geos::geom::LineString const *>(tmp);
                BOOST_ASSERT(linestring);
                linestrings.push_back(linestring);
            }
            break;
        }

        default:
            throw std::runtime_error("Unknown type of intersection.");
        }
            
        // Discretize the boundary to synthesize contact normals.
        contacts->clear();
        BOOST_FOREACH (geos::geom::LineString const *linestring, linestrings) {
            double progress = 0;
            double segment_start = 0;

            for (size_t i = 0; i < linestring->getNumPoints() - 1; ++i) {
                Eigen::Vector3d const p1 = toEigen(linestring->getCoordinateN(i));
                Eigen::Vector3d const p2 = toEigen(linestring->getCoordinateN(i + 1));
                Eigen::Vector3d const normal = orthgonal(p2 - p1).normalized();
                double const segment_length = (p2 - p1).norm();

                while (progress < segment_start + segment_length) {
                    double const r = (progress - segment_start) / segment_length;
                    *contacts += kenv::Contact(r * p1 + (1 - r) * p2, normal);
                    progress += resolution;
                }
                segment_start += segment_length;
            }
        }
        return true;
    }
    // Just check for collision. We don't need to compute any intermediate results.
    else {
        return geom1->intersects(geom2);
    }
}

std::vector<Link::Ptr> PolygonalObject::getLinks(void) const
{
    std::vector<Link::Ptr> links;
    links.reserve(links_.size());

    BOOST_FOREACH (PolygonalLink::Ptr link, links_) {
        links.push_back(link);
    }
    return links;
}

Link::Ptr PolygonalObject::getLink(std::string const name) const
{
    throw std::runtime_error("not implemented");
}

Eigen::Affine3d PolygonalObject::getTransform(void) const
{
    Eigen::Affine2d const pose_2d = base_link_->pose();
    Eigen::Affine3d pose_3d = Eigen::Affine3d::Identity();
    pose_3d.matrix().block<2, 2>(0, 0) = pose_2d.matrix().block<2, 2>(0, 0);
    pose_3d.translation().head<2>() = pose_2d.translation();
    return pose_3d;
}

void PolygonalObject::setTransform(Eigen::Affine3d const &pose_3d)
{
    if (std::fabs(pose_3d.matrix()(2, 2) - 1) > 1e-6) {
        throw std::runtime_error("Rotation is not purely around the z-axis.");
    }
    
    Eigen::Affine2d pose_2d = Eigen::Affine2d::Identity();
    pose_2d.matrix().block<2, 2>(0, 0) = pose_3d.matrix().block<2, 2>(0, 0);
    pose_2d.translation() = pose_3d.translation().head<2>();
    base_link_->set_pose(pose_2d);
}

Eigen::VectorXd PolygonalObject::getDOFValues(void) const
{
    Eigen::VectorXd dof_values(joints_.size());
    for (size_t i = 0; i < joints_.size(); ++i) {
        dof_values[i] = joints_[i]->angle();
    }
    return dof_values;
}

void PolygonalObject::setDOFValues(Eigen::VectorXd const &dof_values)
{
    BOOST_ASSERT(dof_values.size() == static_cast<int>(joints_.size()));
    for (size_t i = 0; i < joints_.size(); ++i) {
        // FIXME: This triggers |joints_| updates. It only needs to trigger one.
        joints_[i]->set_angle(dof_values[i]);
    }
}

void PolygonalObject::setColor(Eigen::Vector4d const &color)
{
    color_ = color;
}

Eigen::Vector4d PolygonalObject::getColor() const
{
    return color_;
}

void PolygonalObject::setTransparency(double p)
{
    BOOST_ASSERT(0 <= p && p <= 1);
    transparency_ = p;
}

double PolygonalObject::getTransparency() const
{
    return transparency_;
}

::PolygonalLink::Ptr PolygonalObject::getBaseLink() const
{
    return base_link_;
}

geos::geom::Geometry *PolygonalObject::getGeometry() const
{
    geos::geom::GeometryFactory const *geom_factory = geos::geom::GeometryFactory::getDefaultInstance();
    std::vector<geos::geom::Geometry *> *geoms = new std::vector<geos::geom::Geometry *>;

    BOOST_FOREACH (Link::Ptr link, getLinks()) {
        PolygonalLink::Ptr polygonal_link = boost::dynamic_pointer_cast<PolygonalLink>(link);
        BOOST_ASSERT(polygonal_link);

        geos::geom::Geometry *geom = polygonal_link->getGeometry();
        geoms->push_back(geom);
    }

    geos::geom::GeometryCollection *geom_collection = geom_factory->createGeometryCollection(geoms);
    return geom_collection->buffer(0);
    // TODO: Does this leak memory?
}

std::vector<geos::geom::Geometry *> PolygonalObject::getSensors() const
{
    std::vector<geos::geom::Geometry *> object_sensors;

    BOOST_FOREACH (Link::Ptr link, getLinks()) {
        PolygonalLink::Ptr polygonal_link = boost::dynamic_pointer_cast<PolygonalLink>(link);
        BOOST_ASSERT(polygonal_link);

        std::vector<geos::geom::Geometry *> link_sensors = polygonal_link->getSensors();
        object_sensors.insert(object_sensors.end(), link_sensors.begin(), link_sensors.end());
    }

    return object_sensors;
}

/*
 * PolygonalEnvironment
 */
Object::Ptr PolygonalEnvironment::getObject(std::string const &name)
{
    BOOST_AUTO(it, objects_.find(name));
    if (it != objects_.end()) {
        return it->second;
    } else {
        return Object::Ptr();
    }
}

std::vector<Object::Ptr> PolygonalEnvironment::getObjects() const
{
    std::vector<Object::Ptr> objects;
    objects.reserve(objects_.size());

    std::string name;
    PolygonalObject::Ptr object;
    BOOST_FOREACH (boost::tie(name, object), objects_) {
        objects.push_back(object);
    }
    return objects;
}

Object::Ptr PolygonalEnvironment::createObject(std::string const &type, std::string const &name, bool anonymous)
{
    if (objects_.find(name) != objects_.end()) {
        throw std::runtime_error(boost::str(
            boost::format("There is already an object named [%s].") % name));
    }

    std::ifstream fin(type.c_str());
    YAML::Parser parser(fin);
    YAML::Node document;
    if (!parser.GetNextDocument(document)) {
        throw std::runtime_error(boost::str(
            boost::format("Unable to load YAML from [%s].") % type));
    }

    ::PolygonalLink::Ptr polygonal_link = boost::make_shared< ::PolygonalLink>();
    polygonal_link->deserialize(document);
    polygonal_link->update();

    PolygonalObject::Ptr object = boost::make_shared<PolygonalObject>(type, name, shared_from_this(), polygonal_link);
    objects_[name] = object;
    return object;

}

void PolygonalEnvironment::remove(Object::Ptr object)
{
    size_t const num_removed = objects_.erase(object->getName());
    if (num_removed == 0) {
        throw std::runtime_error(boost::str(
            boost::format("There is no object named [%s].") % object->getName()));
    }
}

Handle PolygonalEnvironment::drawLine(Eigen::Vector3d const &start, Eigen::Vector3d const &end,
                                      double width, Eigen::Vector4d const &color)
{
    throw std::runtime_error("not implemented");
}

Handle PolygonalEnvironment::drawLineStrip(std::vector<Eigen::Vector3d> const &points,
                                           double width, Eigen::Vector4d const &color)
{
    throw std::runtime_error("not implemented");
}

Handle PolygonalEnvironment::drawLineList(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > const &lines,
                                          double width, Eigen::Vector4d const &color)
{
    throw std::runtime_error("not implemented");
}

Handle PolygonalEnvironment::drawArrow(Eigen::Vector3d const &start, Eigen::Vector3d const &end,
                                       double width, Eigen::Vector4d const &color)
{
    throw std::runtime_error("not implemented");
}

Handle PolygonalEnvironment::drawPoints(std::vector<Eigen::Vector3d> const &points,
                                        float point_size, Eigen::Vector4d const &color)
{
    throw std::runtime_error("not implemented");
}

Handle PolygonalEnvironment::drawPlane(Eigen::Affine3d const &origin, float width, float height,
                                       boost::multi_array<float,3> const &texture)
{
    throw std::runtime_error("not implemented");
}

}
