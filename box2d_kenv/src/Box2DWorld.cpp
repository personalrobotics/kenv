#include <fstream>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <yaml-cpp/yaml.h>
#include "Box2DBody.h"
#include "Box2DFactory.h"
#include "Box2DWorld.h"

using boost::make_shared;
using boost::format;
using boost::str;

typedef std::map<std::string, box2d_kenv::Box2DBodyPtr>::iterator BodyIterator;

namespace box2d_kenv {

Box2DWorld::Box2DWorld(double scale)
    : b2_world_(b2Vec2(0., 0.))
    , scale_(scale)
{
    BOOST_ASSERT(scale > 0.);
}

b2World *Box2DWorld::b2_world()
{
    return &b2_world_;
}

double Box2DWorld::scale() const
{
    return scale_;
}

Box2DBodyPtr Box2DWorld::CreateBody(std::string const &name,
                                    std::istream &stream)
{
#ifdef YAMLCPP_NEWAPI
    YAML::Node node = YAML::Load(stream);
#else
    YAML::Parser parser(stream);
    YAML::Node node;
    parser.GetNextDocument(node);
#endif

    Box2DFactory factory(shared_from_this());
    Box2DBodyPtr const body = factory.CreateBody(name, node);

    std::pair<BodyIterator, bool> const result = bodies_.insert(
        std::make_pair(name, body));
    if (!result.second) {
        throw std::runtime_error(
            str(format("There is already a body named '%s'.") % name));
    }
    return body;
}

Box2DBodyPtr Box2DWorld::CreateBody(std::string const &name,
                                    std::string const &path)
{
    std::ifstream stream(path.c_str());

    if (!stream.good()) {
        throw std::runtime_error(
            str(format("Unable to read body '%s' from '%s'.") % name % path));
    }

    return CreateBody(name, stream);
}

Box2DBodyPtr Box2DWorld::CreateEmptyBody(std::string const &name)
{
    Box2DFactory factory(shared_from_this());
    Box2DBodyPtr const body = factory.CreateEmptyBody(name);
    
    std::pair<BodyIterator, bool> const result = bodies_.insert(
        std::make_pair(name, body));
    if (!result.second) {
        throw std::runtime_error(
            str(format("There is already a body named '%s'.") % name));
    }
    return body;
}

}
