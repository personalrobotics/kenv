#include "Box2DBody.h"
#include "Box2DWorld.h"

namespace box2d_kenv {

Box2DBody::Box2DBody(Box2DWorldPtr const &world, std::string const &name)
    : name_(name)
    , world_(world)
{
    BOOST_ASSERT(world);
}

Box2DWorldPtr Box2DBody::world() const
{
    return world_.lock();
}

}
