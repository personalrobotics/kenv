#ifndef BOX2DBODY_H_
#define BOX2DBODY_H_
#include "box2d_kenv.hh"

namespace box2d_kenv {

class Box2DBody {
public:
    Box2DBody(Box2DWorldPtr const &world, std::string const &name);
    Box2DWorldPtr world() const;

private:
    std::string name_;
    Box2DWorldWeakPtr world_;
    Box2DLinkPtr root_link_;

    friend class Box2DFactory;
};

}

#endif
