#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <Eigen/Dense>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Time.hpp>
#include "Box2DBody.h"
#include "Box2DLink.h"
#include "Box2DJoint.h"
#include "Box2DWorld.h"
#include "Box2DVisualizer.h"
#include "Box2DSensor.h"
#include "Box2DSensorMonitor.h"

using box2d_kenv::Box2DBodyPtr;
using box2d_kenv::Box2DLinkPtr;
using box2d_kenv::Box2DJointPtr;
using box2d_kenv::Box2DWorld;
using box2d_kenv::Box2DWorldPtr;
using box2d_kenv::Box2DSensorPtr;
using box2d_kenv::Box2DVisualizer;
using box2d_kenv::Box2DSensorMonitor;

static void SetInertiaRecursive(Box2DBodyPtr const &body,
                                double mass, double rotational_inertia)
{
    BOOST_FOREACH (Box2DLinkPtr const &link, body->links()) {
        link->set_inertia(mass, rotational_inertia);
    }
}

int main(int argc, char **argv)
{
    sf::Time physics_period = sf::milliseconds(10);
    unsigned int render_skip = 3;
    double render_scale = 1000.;
    double physics_scale = 1000.;
    unsigned int velocity_iterations = 10;
    unsigned int position_iterations = 5;

    unsigned int window_width = 1280;
    unsigned int window_height = 720;
    double window_scale = 1000.;
    std::string window_name = "Box2D Test";
    std::string hand_path = "src/kenv/box2d_kenv/data/barretthand.object.yaml";
    std::string object_path = "src/kenv/box2d_kenv/data/ricepilaf.object.yaml";

    Eigen::Affine2d initial_hand_pose = Eigen::Affine2d::Identity();
    Eigen::Vector3d hand_twist(-0.01, 0., 0.);
    double hand_finger_velocity = 0.3;

    Eigen::Affine2d initial_object_pose = Eigen::Affine2d::Identity();
    initial_object_pose.pretranslate(Eigen::Vector2d(-0.2, 0.1));

    // Setup the physics simulator.
    Box2DWorldPtr const world = boost::make_shared<Box2DWorld>(physics_scale);
    Box2DSensorMonitor const sensor_monitor(world);
    Box2DBodyPtr const ground_body = world->CreateEmptyBody("ground");
    Box2DBodyPtr const object_body = world->CreateBody("object", object_path);
    Box2DBodyPtr const hand_body = world->CreateBody("hand", hand_path);

    hand_body->CreateSensors("data/barretthand_fingertip.sensors.yaml");
    //SetInertiaRecursive(hand_body, 1000., 10000.);

    object_body->set_pose(initial_object_pose);
    object_body->root_link()->enable_friction(ground_body->root_link());
    object_body->root_link()->set_friction(0.5, 0.05);
    //SetInertiaRecursive(object_body, 0.1, 0.1);

    // Create a window.
    unsigned int const bit_depth = sf::VideoMode::getDesktopMode().bitsPerPixel;
    sf::VideoMode video_mode(window_width, window_height, bit_depth);
    sf::RenderWindow window(video_mode, window_name);
    window.setVerticalSyncEnabled(true);

    b2World *b2_world = world->b2_world();
    Box2DVisualizer b2_visualizer(&window, render_scale / physics_scale,
                                  window_width / 2., window_height / 2.);
    b2_visualizer.SetFlags(
        b2Draw::e_shapeBit
      | b2Draw::e_jointBit
      | b2Draw::e_centerOfMassBit
    );
	b2_world->SetDebugDraw(&b2_visualizer);

    sf::Clock clock;
    unsigned int iteration = 0;
    double t = 0.;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }

        // Manually advance the hand to prevent drift.
        // TODO: This ignores the angular component of the twist.
        Eigen::Affine2d hand_pose = initial_hand_pose;
        hand_pose.pretranslate(t * hand_twist.head<2>());
        hand_body->set_pose(hand_pose);
        hand_body->set_twist(hand_twist);

        BOOST_FOREACH (Box2DJointPtr const &joint, hand_body->joints()) {
            b2RevoluteJoint *const b2_joint = joint->b2_joint();
            double const min_limit = std::min(
                std::max(b2_joint->GetJointAngle(), b2_joint->GetLowerLimit()),
                b2_joint->GetUpperLimit()
            );
            double const max_limit = b2_joint->GetUpperLimit();
            b2_joint->SetLimits(min_limit, max_limit);
        }

        hand_body->GetJoint("J01")->set_desired_velocity(hand_finger_velocity);
        hand_body->GetJoint("J21")->set_desired_velocity(hand_finger_velocity);

        // Run a physics timestep.
        object_body->set_twist(Eigen::Vector3d::Zero());
        b2_world->Step(physics_period.asSeconds(), position_iterations,
                       velocity_iterations);

        t += physics_period.asSeconds();

        std::cout << "sensors:";
        BOOST_FOREACH (Box2DSensorPtr const &sensor, hand_body->sensors()) {
            bool const is_active = sensor_monitor.IsSensorActive(sensor);
            std::cout << " " << sensor->name() << " = " << is_active;
        }
        std::cout << std::endl;

        // Update the viewer.
        if (iteration % render_skip == 0) {
            window.clear(sf::Color::Black);
            b2_world->DrawDebugData();
            window.display();

            iteration = 0;
        } else {
            iteration++;
        }

        // Sleep to maintain a constant framerate.
        sf::Time const skew = clock.getElapsedTime();

        if (physics_period > skew) {
            sf::sleep(physics_period - skew);
        }
        clock.restart();
    }

    return 0;
}

