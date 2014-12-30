#ifndef BOX2DWORLD_H_
#define BOX2DWORLD_H_
#include <map>
#include <boost/enable_shared_from_this.hpp>
#include <Box2D/Dynamics/b2World.h>
#include "box2d_kenv.hh"

namespace box2d_kenv {

class Box2DWorld : public boost::enable_shared_from_this<Box2DWorld> {
public:
    Box2DWorld(double scale);

    b2World *b2_world();
    double scale() const;

    Box2DBodyPtr CreateBody(std::string const &name, std::istream &stream);
    Box2DBodyPtr CreateBody(std::string const &name, std::string const &path);

private:
    b2World b2_world_;
    std::map<std::string, Box2DBodyPtr> bodies_;
    double scale_;
};

}

#endif
