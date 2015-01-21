#ifndef BOX2DSENSORMONITOR_H_
#define BOX2DSENSORMONITOR_H_
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <Box2D/Dynamics/b2WorldCallbacks.h>
#include "box2d_kenv.hh"

class b2Contact;
class b2Fixture;

namespace box2d_kenv {

namespace detail {

struct Box2DSensorState {
    void set_contact(b2Fixture const *b2_fixture, bool is_contact);

    bool is_active() const;

private:
    boost::unordered_set<b2Fixture const *> active_fixtures;
};

}

class Box2DSensorMonitor : public b2ContactListener {
public:
    Box2DSensorMonitor(Box2DWorldPtr const &world);
    virtual ~Box2DSensorMonitor();

    bool IsSensorActive(Box2DSensorPtr const &sensor) const;

    virtual void BeginContact(b2Contact *contact);
    virtual void EndContact(b2Contact *contact);

    virtual void PreSolve(b2Contact *contact, b2Manifold const *oldManifold);
    virtual void PostSolve(b2Contact *contact, b2ContactImpulse const *impulse);

private:
    Box2DWorldPtr world_;
    boost::unordered_map<Box2DSensor const *, detail::Box2DSensorState> sensors_;

    bool ProcessContactEvent(b2Fixture *b2_fixture, bool is_contact);
};

}

#endif
