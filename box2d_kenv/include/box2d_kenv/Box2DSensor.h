#ifndef BOX2DSENSOR_H_
#define BOX2DSENSOR_H_
#include "box2d_kenv.hh"

class b2Body;
class b2Fixture;

namespace box2d_kenv {

class Box2DSensor {
public:
    Box2DSensor(Box2DLinkPtr const &parent_link,
                std::vector<b2Fixture *> const &b2_fixtures);

    Box2DBodyPtr parent_body() const;
    Box2DLinkPtr parent_link() const;

    std::vector<b2Fixture *> b2_fixtures();
    std::vector<b2Fixture const *> b2_fixtures() const;

private:
    Box2DLinkWeakPtr parent_link_;
    std::vector<b2Fixture *> b2_fixtures_;
};

}

#endif
