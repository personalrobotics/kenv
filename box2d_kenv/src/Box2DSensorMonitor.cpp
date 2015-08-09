#include <iostream>
#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include "Box2DLink.h"
#include "Box2DSensor.h"
#include "Box2DSensorMonitor.h"
#include "Box2DWorld.h"

namespace box2d_kenv {

namespace detail {

void Box2DSensorState::set_contact(b2Fixture const *b2_fixture,
                                   bool is_contact)
{
    typedef boost::unordered_set<b2Fixture const *>::iterator iterator;
    if (is_contact) {
        std::pair<iterator, bool> const result
            = active_fixtures.insert(b2_fixture);
        BOOST_ASSERT(result.second);
    } else {
        size_t const num_erased = active_fixtures.erase(b2_fixture);
        BOOST_ASSERT(num_erased == 1);
    }
}

bool Box2DSensorState::is_active() const
{
    return !active_fixtures.empty();
}

}

Box2DSensorMonitor::Box2DSensorMonitor(Box2DWorldPtr const &world)
    : world_(world)
{
    BOOST_ASSERT(world);

    world->b2_world()->SetContactListener(this);
}

Box2DSensorMonitor::~Box2DSensorMonitor()
{
    world_->b2_world()->SetContactListener(NULL);
}

bool Box2DSensorMonitor::IsSensorActive(Box2DSensorPtr const &sensor) const
{
    typedef boost::unordered_map<
        Box2DSensor const *, detail::Box2DSensorState
            >::const_iterator const_iterator;

    const_iterator const it = sensors_.find(sensor.get());
    if (it != sensors_.end()) {
        return it->second.is_active();
    } else {
        return false;
    }
}

void Box2DSensorMonitor::BeginContact(b2Contact *contact)
{
    ProcessContactEvent(contact->GetFixtureA(), true);
    ProcessContactEvent(contact->GetFixtureB(), true);
}

void Box2DSensorMonitor::EndContact(b2Contact *contact)
{
    ProcessContactEvent(contact->GetFixtureA(), false);
    ProcessContactEvent(contact->GetFixtureB(), false);
}

void Box2DSensorMonitor::PreSolve(b2Contact *contact,
                                  b2Manifold const *oldManifold)
{
}

void Box2DSensorMonitor::PostSolve(b2Contact *contact,
                                   b2ContactImpulse const *impulse)
{
}

bool Box2DSensorMonitor::ProcessContactEvent(b2Fixture *b2_fixture,
                                             bool is_contact)
{
    // This is not a sensor.
    if (!b2_fixture->IsSensor()) {
        return false;
    }

    // This is not a sensor that we've wrapped. This happen if the user
    // manually created a Box2D sensor object.
    Box2DSensor *const sensor = static_cast<Box2DSensor *>(
        b2_fixture->GetUserData());
    if (!sensor) {
        return false;
    }

    detail::Box2DSensorState &sensor_state = sensors_[sensor];
    sensor_state.set_contact(b2_fixture, is_contact);

    return true;
}

}
