#include <iostream>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>
#include <geos/geom/CoordinateSequenceFactory.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/MultiLineString.h>
#include <yaml-cpp/yaml.h>
#include <kenv/OREnvironment.h>
#include <quasistatic_pushing/QuasistaticPushingModel.h>
#include "PolygonalEnvironment.h"
#include "PolygonalViewer.h"
#include "CSpaceObstacle.h"

namespace po = boost::program_options;

std::vector<bool> SimulateSensors(kenv::Object::Ptr robot, kenv::Object::Ptr obstacle,
                                  double tolerance)
{
    kenv::PolygonalObject::Ptr robot_2d = boost::dynamic_pointer_cast<kenv::PolygonalObject>(robot);
    kenv::PolygonalObject::Ptr obstacle_2d = boost::dynamic_pointer_cast<kenv::PolygonalObject>(obstacle);
    BOOST_ASSERT(robot_2d);
    BOOST_ASSERT(obstacle_2d);

    std::vector<geos::geom::Geometry *> sensors_geom = robot_2d->getSensors();
    geos::geom::Geometry *obstacle_geom = obstacle_2d->getGeometry();
    obstacle_geom = obstacle_geom->buffer(tolerance);

    std::vector<bool> observation(sensors_geom.size());
    for (size_t i = 0; i < sensors_geom.size(); ++i) {
        geos::geom::Geometry *sensor_geom = sensors_geom[i];
        observation[i] = obstacle_geom->intersects(sensor_geom);
    }

    return observation;
}

geos::geom::Geometry *RenderContacts(std::vector<kenv::Contact> const &contacts)
{
    geos::geom::GeometryFactory const *geom_factory = geos::geom::GeometryFactory::getDefaultInstance();
    geos::geom::CoordinateSequenceFactory const *cs_factory = geom_factory->getCoordinateSequenceFactory();
    std::vector<geos::geom::Geometry *> *lines = new std::vector<geos::geom::Geometry *>;

    BOOST_FOREACH (kenv::Contact const &contact, contacts) {
        std::vector<geos::geom::Coordinate> *coords = new std::vector<geos::geom::Coordinate>(2);
        (*coords)[0].x = contact.position[0];
        (*coords)[0].y = contact.position[1];
        (*coords)[1].x = contact.position[0] + 0.01 * contact.normal[0];
        (*coords)[1].y = contact.position[1] + 0.01 * contact.normal[1];

        geos::geom::CoordinateSequence *coord_seq = cs_factory->create(coords);
        geos::geom::LineString *line = geom_factory->createLineString(coord_seq);
        lines->push_back(line);
    }
    return geom_factory->createMultiLineString(lines);
}

geos::geom::Geometry *RenderObservation(kenv::Object::Ptr robot, std::vector<bool> const &observation)
{
    geos::geom::GeometryFactory const *geom_factory = geos::geom::GeometryFactory::getDefaultInstance();

    kenv::PolygonalObject::Ptr robot_2d = boost::dynamic_pointer_cast<kenv::PolygonalObject>(robot);
    BOOST_ASSERT(robot_2d);

    std::vector<geos::geom::Geometry *> sensors_geom = robot_2d->getSensors();
    BOOST_ASSERT(sensors_geom.size() == observation.size());

    std::vector<geos::geom::Geometry *> *render_geoms = new std::vector<geos::geom::Geometry *>;
    for (size_t i = 0; i < observation.size(); ++i) {
        if (observation[i]) {
            render_geoms->push_back(sensors_geom[i]);
        }
    }
    return geom_factory->createGeometryCollection(render_geoms);
}

int main(int argc, char **argv)
{
    double mu, c;
    std::string mode, hand_type, object_type;

    po::options_description desc("./test_polygonal");
    desc.add_options()
        ("mode", po::value<std::string>(&mode)->default_value("simulate"), "mode")
        ("mu", po::value<double>(&mu)->default_value(0.5), "friction coefficient")
        ("c", po::value<double>(&c)->default_value(0.05), "pressure distribution radius")
        ("hand_type", po::value<std::string>(&hand_type)->required(), "type of the hand")
        ("object_type", po::value<std::string>(&object_type)->required(), "type of the object");

    po::positional_options_description p;
    p.add("hand_type", 1);
    p.add("object_type", 1);

    po::variables_map vm;
    try {
        po::store(po::command_line_parser(argc, argv).
        options(desc).positional(p).run(), vm);
        po::notify(vm);
    } catch(std::exception const &e) {
        std::cerr << e.what() << std::endl;
        std::cout << desc << std::endl;
        return 1;
    }

    kenv::PolygonalEnvironment::Ptr env = boost::make_shared<kenv::PolygonalEnvironment>();
    kenv::Object::Ptr hand = env->createObject(hand_type, "hand");
    kenv::Object::Ptr object = env->createObject(object_type, "object");
    hand->setDOFValues(Eigen::Matrix<double, 8, 1>::Zero());
    hand->setColor(Eigen::Vector4d(1, 0, 0, 1));
    object->setColor(Eigen::Vector4d(0, 1, 0, 1));

    kenv::PolygonalObject::Ptr hand_polygonal = boost::dynamic_pointer_cast<kenv::PolygonalObject>(hand);
    kenv::PolygonalObject::Ptr object_polygonal = boost::dynamic_pointer_cast<kenv::PolygonalObject>(object);

    PolygonalViewer viewer(env, "Visualization", 800, 600);

    if (mode == "simulate") {
        kenv::CollisionChecker::Ptr collision_checker = boost::make_shared<kenv::DefaultCollisionChecker>();
        quasistatic_pushing::QuasistaticPushingModel simulator(collision_checker, 0.001, 0.01);
        quasistatic_pushing::Action action(Eigen::Vector2d(-0.001, 0), 0);

        // Arbitrary object pose for testing.
        Eigen::Affine3d object_pose = Eigen::Affine3d::Identity();
        object_pose.pretranslate(Eigen::Vector3d(-0.20, 0.12, 0));
        object->setTransform(object_pose);

        for (;;) {
            simulator.Simulate(hand, object, action, mu, c);
            std::vector<kenv::Contact> contacts;
            hand_polygonal->checkCollision(object_polygonal, &contacts);

            std::vector<bool> observation = SimulateSensors(hand, object, 0.001);

            viewer.ClearCustomGeometry();
            viewer.AddCustomGeometry(RenderContacts(contacts), Eigen::Vector4d(1, 0, 0, 1));
            viewer.AddCustomGeometry(RenderObservation(hand, observation), Eigen::Vector4d(0, 1, 0, 1));
            viewer.SpinOnce();
        }
    } else if (mode == "cobstacle") {
        double const angular_resolution = 10 * (M_PI / 180); 
        size_t const num_angular_samples = static_cast<size_t>(2 * M_PI / angular_resolution);

        YAML::Emitter emitter;
        emitter << YAML::BeginSeq;

        for (size_t i = 0; i < num_angular_samples; ++i) {
            double const angle = i * angular_resolution;
            Eigen::Affine3d object_pose = Eigen::Affine3d::Identity();
            object_pose.rotate(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
            object->setTransform(object_pose);

            geos::geom::Geometry *hand_geometry = hand_polygonal->getGeometry();
            geos::geom::Geometry *object_geometry = object_polygonal->getGeometry();
            geos::geom::Geometry *cspace_obstacle = ComputeCSpaceObstacle(hand_geometry, object_geometry);
            viewer.AddCustomGeometry(cspace_obstacle, Eigen::Vector4d(0, 1, 0, 1));

            emitter << YAML::BeginMap
                    << YAML::Key << "angle" << YAML::Value << angle
                    << YAML::Key << "manifold" << YAML::Value
                        << YAML::LocalTag("WKT") << cspace_obstacle->toText()
                    << YAML::EndMap;
        }

        emitter << YAML::EndSeq;
        std::cout << emitter.c_str() << std::endl;

        viewer.Spin();
    } else {
        std::cerr << "unknown mode [" << mode << "]" << std::endl;
        return 1;
    }
    return 0;
}
