#include <boost/make_shared.hpp>
#include <Eigen/Dense>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Time.hpp>
#include "Box2DBody.h"
#include "Box2DWorld.h"
#include "Box2DVisualizer.h"

using box2d_kenv::Box2DBodyPtr;
using box2d_kenv::Box2DWorld;
using box2d_kenv::Box2DWorldPtr;
using box2d_kenv::Box2DVisualizer;

int main(int argc, char **argv)
{
    sf::Time render_period = sf::milliseconds(2);
    double render_scale = 1000.;
    double physics_scale = 1000.;
    unsigned int physics_multiplier = 10;
    unsigned int velocity_iterations = 10;
    unsigned int position_iterations = 5;

    unsigned int window_width = 1280;
    unsigned int window_height = 720;
    double window_scale = 1000.;
    std::string window_name = "Box2D Test";
    std::string hand_path = "data/barretthand_twofinger.object.yaml";
    std::string object_path = "data/ricepilaf.object.yaml";

    // Setup the physics simulator.
    Box2DWorldPtr const world = boost::make_shared<Box2DWorld>(physics_scale);
    Box2DBodyPtr const hand_body = world->CreateBody("hand", hand_path);
    Box2DBodyPtr const object_body = world->CreateBody("object", object_path);
    b2World *b2_world = world->b2_world();

    Eigen::Affine2d object_pose = Eigen::Affine2d::Identity();
    object_pose.pretranslate(Eigen::Vector2d(-0.2, 0.));

    object_body->set_pose(object_pose);
    hand_body->set_twist(Eigen::Vector3d(-0.01, 0., 0.));

    // Create a window.
    sf::VideoMode const video_mode(window_width, window_height);
    sf::RenderWindow window(video_mode, window_name);
    window.setVerticalSyncEnabled(true);

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

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }

        // Run a physics timestep.
        if (iteration % physics_multiplier == 0) {
            float const physics_period = physics_multiplier * render_period.asSeconds();
            b2_world->Step(physics_period, position_iterations, velocity_iterations);
            iteration = 0;
        }

        window.clear(sf::Color::Black);
        b2_world->DrawDebugData();
        window.display();

        // Sleep to maintain a constant framerate.
        sf::Time const skew = clock.getElapsedTime();

        if (render_period > skew) {
            sf::sleep(render_period - skew);
        }

        clock.restart();
    }

    return 0;
}

