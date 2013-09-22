#include <iostream>
#include <vector>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/format.hpp>

#include <Eigen/Dense>

#include <geos/geom/Coordinate.h>
#include <geos/geom/CoordinateSequence.h>
#include <geos/geom/CoordinateSequenceFactory.h>
#include <geos/geom/Polygon.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/MultiPoint.h>
#include <geos/operation/union/CascadedPolygonUnion.h>

#include <openrave-core.h>
#include <openrave/openrave.h>

#include "or_conversions.h"
#include "PolygonalLink.h"

using namespace boost::assign;
using geos::geom::Coordinate;
using geos::geom::CoordinateSequence;
using geos::geom::CoordinateSequenceFactory;
using geos::geom::Geometry;
using geos::geom::GeometryFactory;
using geos::geom::MultiPoint;
using geos::geom::LinearRing;
using geos::geom::Polygon;

GeometryFactory const *geom_factory;
CoordinateSequenceFactory const *cs_factory;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

Eigen::Affine2d to2D(Eigen::Affine3d const &pose_3d)
{
    Eigen::Affine2d pose = Eigen::Affine2d::Identity();
    pose.translation() = pose_3d.translation().head<2>();
    return pose;
}

template <class Point>
Polygon *createPolygon(std::vector<Point> const &vertices)
{
    GeometryFactory const *geom_factory = GeometryFactory::getDefaultInstance();
    CoordinateSequenceFactory const *cs_factory = geom_factory->getCoordinateSequenceFactory();

    std::vector<Coordinate> *coords = new std::vector<Coordinate>;
    std::vector<Geometry *> *holes = new std::vector<Geometry *>;
    coords->reserve(vertices.size() + 1);

    // Add the polygon's vertices.
    BOOST_FOREACH (Point const &vertex, vertices) {
        Coordinate coord(vertex.first, vertex.second);
        coords->push_back(coord);
    }

    // Close the loop. This is necessary to create a LinearRing.
    Coordinate coord(vertices[0].first, vertices[0].second);
    coords->push_back(coord);

    CoordinateSequence *coord_seq = cs_factory->create(coords);
    LinearRing *linear_ring = geom_factory->createLinearRing(coord_seq);
    return geom_factory->createPolygon(linear_ring, holes);
}

void PrintPolygonalLink(std::ostream &stream, PolygonalLink const &link, size_t depth = 0)
{
    for (size_t i = 0; i < depth; ++i) {
        std::cout << "-";
    }
    stream << " Link[" << link.name() << "]\n";

    BOOST_FOREACH (PolygonalLink::Ptr child_link, link.children()) {
        PrintPolygonalLink(stream, *child_link, depth + 1);
    }
}

PolygonalLink::Ptr flattenLink(OpenRAVE::KinBody::LinkConstPtr link,
                               Eigen::Affine2d const &Tworld_joint = Eigen::Affine2d::Identity(),
                               OpenRAVE::KinBody::LinkConstPtr parent_link = OpenRAVE::KinBody::LinkConstPtr())
{
    // Relative transformation w.r.t. this link's parent.
    OpenRAVE::Transform Tworld_link_or = link->GetTransform();
    Eigen::Affine2d const Tworld_link = to2D(toEigen(Tworld_link_or));

    // Flatten the geometry to 2D polygons.
    std::vector<geos::geom::Polygon *> *polygons = new std::vector<geos::geom::Polygon *>;
    polygons->reserve(link->GetGeometries().size());

    BOOST_FOREACH (OpenRAVE::KinBody::Link::GeometryPtr geom, link->GetGeometries()) {
        OpenRAVE::Transform const Tlink_geom_or = geom->GetTransform();

        if (geom->GetType() != OpenRAVE::GT_TriMesh) {
            throw std::runtime_error(boost::str(
                boost::format("Link[%s]: Only trimesh geometry is supported.") % link->GetName()));
        }

        // Transform the vertices into the world frame.
        OpenRAVE::TriMesh const &mesh = geom->GetCollisionMesh();
        std::vector<Coordinate> *coords = new std::vector<Coordinate>;
        coords->reserve(coords->size() + mesh.vertices.size());
        BOOST_FOREACH (OpenRAVE::Vector const &vertex_geom, mesh.vertices) {
            OpenRAVE::Vector const vertex_link = Tworld_link_or * Tlink_geom_or * vertex_geom;
            coords->push_back(Coordinate(vertex_link.x, vertex_link.y));
        }

        // Approximate the geometry with the convex hull of its trimesh vertices.
        CoordinateSequence *coord_seq = cs_factory->create(coords);
        MultiPoint *points = geom_factory->createMultiPoint(*coord_seq);
        delete coord_seq;

        geos::geom::Geometry *geom = points->convexHull();
        geos::geom::Polygon *polygon = dynamic_cast<geos::geom::Polygon *>(geom);
        BOOST_ASSERT(polygon);
        BOOST_ASSERT(polygon->isValid());
        polygons->push_back(polygon);
    }

    // Union all of the geometries into a single multipolygon. Using a union is
    // necessary to properly handle self-intersection.
    geos::geom::MultiPolygon *world_polygons;
    geos::operation::geounion::CascadedPolygonUnion polygon_union(polygons);
    geos::geom::Geometry *world_geoms = polygon_union.Union();

    switch (world_geoms->getGeometryTypeId()) {
    case geos::geom::GEOS_POLYGON: {
        std::vector<geos::geom::Geometry *> *tmp = new std::vector<geos::geom::Geometry *>;
        tmp->push_back(polygons->at(0));
        world_polygons = geom_factory->createMultiPolygon(tmp);
        break;
    }

    case geos::geom::GEOS_MULTIPOLYGON: {
        world_polygons = dynamic_cast<geos::geom::MultiPolygon *>(world_geoms);
        break;
    }

    default:
        throw std::runtime_error(boost::str(
            boost::format("Link[%s]: Geometry is not polygonal. Is the geometry degenerate?")));
    }
    // TODO: properly free the memory
    BOOST_ASSERT(world_polygons);
    BOOST_ASSERT(world_polygons->isValid());

    // Transform the geometry into the link frame.
    geos::geom::Geometry *geometry = TransformGeometry(world_polygons, Tworld_link.inverse());
    BOOST_ASSERT(geometry->isValid());
    delete world_polygons;

    // Create the new link.
    PolygonalLink::Ptr flat_link = boost::make_shared<PolygonalLink>(link->GetName(),
        Tworld_joint.inverse() * Tworld_link, geometry);

    // Recursively construct child links.
    std::vector<OpenRAVE::KinBody::JointPtr> all_joints;
    std::vector<OpenRAVE::KinBody::JointPtr> const normal_joints = link->GetParent()->GetJoints();
    std::vector<OpenRAVE::KinBody::JointPtr> const mimic_joints = link->GetParent()->GetPassiveJoints();
    all_joints.reserve(normal_joints.size() + mimic_joints.size());
    all_joints.insert(all_joints.end(), normal_joints.begin(), normal_joints.end());
    all_joints.insert(all_joints.end(), mimic_joints.begin(), mimic_joints.end());
    
    BOOST_FOREACH (OpenRAVE::KinBody::JointPtr joint, all_joints) {
        // Choose the other link attached to the joint.
        OpenRAVE::KinBody::LinkPtr child_link;
        if (link == joint->GetFirstAttached()) {
            child_link = joint->GetSecondAttached();
        } else if (link == joint->GetSecondAttached()) {
            child_link = joint->GetFirstAttached();
        } else {
            continue;
        }

        // Avoid entering a cycle.
        if (child_link == parent_link) {
            continue;
        }

        // Find the joint origin frame.
        // TODO: Include the current angle.
        int const direction = sgn(joint->GetAxis()[2]);
        Eigen::Affine2d Tworld_origin = Eigen::Affine2d::Identity();
        Tworld_origin.rotate(Eigen::Rotation2Dd(0.0));
        Tworld_origin.pretranslate(toEigen3(joint->GetAnchor()).head<2>());

        // Create the child link.
        PolygonalLink::Ptr flat_child_link = flattenLink(child_link, Tworld_origin, link);

        // Create the joint.
        Eigen::Affine2d const Tlink_origin = Tworld_link.inverse() * Tworld_origin;
        flat_link->AddJoint(joint->GetName(), flat_child_link, Tlink_origin.translation(), direction);
        flat_link->joints()[0]->set_angle(0.9);
    }
    return flat_link;
}

int main(int argc, char **argv)
{
    namespace po = boost::program_options;

    std::string type;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("type", po::value<std::string>(&type), "object type")
        ("help", "produce help message")
    ;

    po::positional_options_description p;
    p.add("type", 1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    po::notify(vm);    

    if (vm.count("help")) {
        std::cerr << desc << std::endl;
        return 1;
    }

    // Load the KinBody file into OpenRAVE.
    namespace OR = OpenRAVE;
    OR::RaveInitialize(true);
    OR::EnvironmentBasePtr env = OR::RaveCreateEnvironment();
    OR::KinBodyPtr body = env->ReadKinBodyURI(OR::KinBodyPtr(), type);
    if (!body) {
        std::cerr << "Error: Unable to create KinBody of type [" << type << "]." << std::endl;
        return 1;
    }
    body->SetName("hand");
    env->Add(body, true);

    // FIXME: The hand offset should be specified as a parameter.
    OR::TransformMatrix pose;
#if 0
    pose.rotfrommat( 0, 0, -1,
                    -1, 0,  0,
                     0, 1,  0);
#endif

    body->SetTransform(pose);

    // Flatten the KinBody into polygons.
    OR::KinBody::LinkPtr base_link = body->GetLinks()[0];
    if (!base_link) {
        std::cerr << "Error: KinBody contains no links." << std::endl;
        return 1;
    }
    geom_factory = GeometryFactory::getDefaultInstance();
    cs_factory = geom_factory->getCoordinateSequenceFactory();
    PolygonalLink::Ptr flat_base_link = flattenLink(base_link);

    YAML::Emitter emitter;
    emitter << *flat_base_link;
    std::cout << emitter.c_str() << std::endl;

    OR::RaveDestroy();
    return 0;
}
