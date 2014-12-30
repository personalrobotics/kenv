#include <boost/make_shared.hpp>
#include <kenv/ObjectPool.h>
#include <polygonal_kenv/PolygonalEnvironment.h>
#include "transition/Box2DTransitionModel.h"
#include "util/Box2DVisualizer.h"

using manifold_pf3::util::Box2DVisualizer;
using contact_manipulation::continuous::Box2DTransitionModel;

int main(int argc, char **argv)
{
    sf::Time render_period = sf::milliseconds(2);
    unsigned int physics_multiplier = 10;
    unsigned int velocity_iterations = 10;
    unsigned int position_iterations = 5;

    unsigned int window_width = 1280;
    unsigned int window_height = 720;
    double window_scale = 1000.;
    std::string window_name = "Box2D Test";
    std::string hand_type = "data/barretthand_twofinger.object.yaml";
    std::string object_type = "data/ricepilaf.object.yaml";

    auto const env = boost::make_shared<kenv::PolygonalEnvironment>();
    auto const pool = boost::make_shared<kenv::MultiObjectPool>(env);

    Box2DTransitionModel transition_model(pool, hand_type, object_type);
    b2World *b2_world = transition_model.GetBox2DWorld();

    Box2DVisualizer b2_visualizer(
        window_width, window_height,
        window_width / -2., window_height / -2.,
        1., window_name
    );
    b2_visualizer.SetFlags(
        b2Draw::e_shapeBit
      | b2Draw::e_jointBit
      | b2Draw::e_centerOfMassBit
    );
	b2_world->SetDebugDraw(&b2_visualizer);

    sf::RenderWindow &window = b2_visualizer.window();
    window.setVerticalSyncEnabled(true);

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

