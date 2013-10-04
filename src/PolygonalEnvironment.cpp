#include <fstream>
#include <boost/assert.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/typeof/typeof.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/make_shared.hpp>
#include <geos/geom/Geometry.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/LineString.h>
#include <geos/geom/MultiLineString.h>
#include <geos/geom/Polygon.h>
#include <geos/geom/Point.h>
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

boost::shared_ptr<geos::geom::Geometry const> PolygonalLink::getGeometry() const
{
    return internal_link_->geometry();
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

    boost::shared_ptr<geos::geom::Geometry const> geom1 = getGeometry();
    boost::shared_ptr<geos::geom::Geometry const> geom2 = other->getGeometry();

    if (links) {
        links->clear();

        std::vector<kenv::Link::Ptr> links1 = getLinks();
        std::vector<kenv::Link::Ptr> links2 = other->getLinks();

        BOOST_FOREACH (kenv::Link::Ptr link1, links1)
        BOOST_FOREACH (kenv::Link::Ptr link2, links2) {
            PolygonalLink::Ptr polygonal_link1 = boost::dynamic_pointer_cast<PolygonalLink>(link1);
            PolygonalLink::Ptr polygonal_link2 = boost::dynamic_pointer_cast<PolygonalLink>(link2);
            boost::shared_ptr<geos::geom::Geometry const> geom_link1 = polygonal_link1->getGeometry();
            boost::shared_ptr<geos::geom::Geometry const> geom_link2 = polygonal_link2->getGeometry();

            if (geom_link1->intersects(geom_link2.get())) {
                links->push_back(std::make_pair(link1, link2));
            }
        }
    }

    if (contacts) {
        std::vector<geos::geom::LineString const *> linestrings;

        // Compute the intersection line(s).
        geos::geom::Geometry *boundary = geom1->getBoundary()->intersection(geom2.get());
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
        return geom1->intersects(geom2.get());
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
    // TODO: Use std::map instead of a linear search.
    BOOST_FOREACH (PolygonalLink::Ptr link, links_) {
        if (link->getName() == name) {
            return link;
        }
    }
    throw std::runtime_error(boost::str(
        boost::format("There is no link named [%s].") % name));
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

boost::shared_ptr<geos::geom::Geometry const> PolygonalObject::getGeometry() const
{
    geos::geom::GeometryFactory const *geom_factory = geos::geom::GeometryFactory::getDefaultInstance();
    std::vector<geos::geom::Geometry *> *geoms = new std::vector<geos::geom::Geometry *>;

    BOOST_FOREACH (Link::Ptr link, getLinks()) {
        PolygonalLink::Ptr polygonal_link = boost::dynamic_pointer_cast<PolygonalLink>(link);
        BOOST_ASSERT(polygonal_link);

        boost::shared_ptr<geos::geom::Geometry const> geom = polygonal_link->getGeometry();
        geoms->push_back(geom->clone());
    }

    geos::geom::GeometryCollection *geom_collection = geom_factory->createGeometryCollection(geoms);
    boost::shared_ptr<geos::geom::Geometry> geom_buffered(geom_collection->buffer(0));
    delete geom_collection;
    return geom_buffered;
}

/*
 * PolygonalEnvironment
 */
PolygonalEnvironment::PolygonalEnvironment()
    : geom_factory_(geos::geom::GeometryFactory::getDefaultInstance())
    , coords_factory_(geom_factory_->getCoordinateSequenceFactory())
{
}

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
    if (anonymous) {
        // Extract the current index from the desired name.
        size_t last_index = name.find_last_not_of("0123456789");
        if (last_index == std::string::npos) {
            last_index  = 0;
        } else {
            last_index += 1;
        }
        std::string const base_name = name.substr(0, last_index);
        std::string const old_index_str = name.substr(last_index);
        size_t const old_index = (old_index_str.empty()) ? 1 : boost::lexical_cast<size_t>(old_index_str);

        std::string unique_name = name;
        for (size_t new_index = old_index; objects_.count(unique_name) != 0; new_index++) {
            unique_name = boost::str(boost::format("%s%d") % base_name % new_index);
        }
        return createObject(type, unique_name, false);
    } else {
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
}

void PolygonalEnvironment::remove(Object::Ptr object)
{
    size_t const num_removed = objects_.erase(object->getName());
    if (num_removed == 0) {
        throw std::runtime_error(boost::str(
            boost::format("There is no object named [%s].") % object->getName()));
    }
}

Handle PolygonalEnvironment::drawGeometry(geos::geom::Geometry *geom, Eigen::Vector4d const &color)
{
    boost::shared_ptr<ColoredGeometry> viz_geom = boost::make_shared<ColoredGeometry>(geom, color);
    visualization_.push_back(viz_geom);
    return viz_geom;
}

Handle PolygonalEnvironment::drawLine(Eigen::Vector3d const &start, Eigen::Vector3d const &end,
                                      double width, Eigen::Vector4d const &color)
{
    std::vector<Eigen::Vector3d> points;
    points.push_back(start);
    points.push_back(end);
    return drawLineStrip(points, width, color);
}

Handle PolygonalEnvironment::drawLineStrip(std::vector<Eigen::Vector3d> const &points,
                                           double width, Eigen::Vector4d const &color)
{
    geos::geom::CoordinateSequence *coords = coords_factory_->create(points.size(), 2);
    
    for (size_t i = 0; i < points.size(); ++i) {
        coords->setAt(toGeos2D(points[i]), i);
    }
    return drawGeometry(geom_factory_->createLineString(coords), color);
}

Handle PolygonalEnvironment::drawLineList(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > const &lines,
                                          double width, Eigen::Vector4d const &color)
{
    std::vector<geos::geom::Geometry *> *line_geoms = new std::vector<geos::geom::Geometry *>;
    line_geoms->reserve(lines.size());

    typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> EndPoints;
    BOOST_FOREACH (EndPoints const &endpoints, lines) {
        geos::geom::CoordinateSequence *coords = coords_factory_->create(2, 2);
        coords->setAt(toGeos2D(endpoints.first), 0);
        coords->setAt(toGeos2D(endpoints.second), 0);

        geos::geom::LineString *line = geom_factory_->createLineString(coords);
        line_geoms->push_back(line);
    }
    return drawGeometry(geom_factory_->createMultiLineString(line_geoms), color);
}

Handle PolygonalEnvironment::drawArrow(Eigen::Vector3d const &start, Eigen::Vector3d const &end,
                                       double width, Eigen::Vector4d const &color)
{
    throw std::runtime_error("not implemented");
}

Handle PolygonalEnvironment::drawPoints(std::vector<Eigen::Vector3d> const &points,
                                        float point_size, Eigen::Vector4d const &color)
{
    std::vector<geos::geom::Geometry *> *point_geoms = new std::vector<geos::geom::Geometry *>;
    point_geoms->reserve(points.size());

    BOOST_FOREACH (Eigen::Vector3d const &point, points) {
        geos::geom::Point *point_geom = geom_factory_->createPoint(toGeos2D(point));
        point_geoms->push_back(point_geom);
    }
    return drawGeometry(geom_factory_->createMultiPoint(point_geoms), color);
}

Handle PolygonalEnvironment::drawPlane(Eigen::Affine3d const &origin, float width, float height,
                                       boost::multi_array<float,3> const &texture)
{
    throw std::runtime_error("not implemented");
}

geos::geom::Coordinate PolygonalEnvironment::toGeos2D(Eigen::Vector3d const &point) const
{
    BOOST_ASSERT(point[2] == 0);
    return geos::geom::Coordinate(point[0], point[1]);
}

std::vector<ColoredGeometry::Ptr> PolygonalEnvironment::getVisualizationGeometry()
{
    std::vector<ColoredGeometry::WeakPtr> new_visualization;
    std::vector<ColoredGeometry::Ptr> geoms;
    geoms.reserve(visualization_.size());
    new_visualization.reserve(visualization_.size());

    for (size_t i = 0; i < visualization_.size(); ++i) {
        ColoredGeometry::Ptr geom = visualization_[i].lock();
        if (geom) {
            geoms.push_back(geom);
            new_visualization.push_back(geom);
        }
    }

    visualization_ = new_visualization;
    return geoms;
}

}
