#include <stdexcept>
#include <boost/foreach.hpp>
#include <Box2D/Dynamics/b2Fixture.h>
#include "Box2DLink.h"
#include "Box2DSensor.h"

namespace box2d_kenv {

Box2DSensor::Box2DSensor(Box2DLinkPtr const &parent_link,
                         std::string const &name,
                         std::vector<b2Fixture *> const &b2_fixtures)
    : parent_link_(parent_link)
    , name_(name)
    , b2_fixtures_(b2_fixtures)
{
    BOOST_ASSERT(parent_link);

    // Add back-references from Box2D.
    BOOST_FOREACH (b2Fixture *const b2_fixture, b2_fixtures) {
        b2_fixture->SetUserData(this);
    }
}

Box2DBodyPtr Box2DSensor::parent_body() const
{
    return parent_link_.lock()->parent_body();
}

Box2DLinkPtr Box2DSensor::parent_link() const
{
    return parent_link_.lock();
}

std::string Box2DSensor::name() const
{
    return name_;
}

std::vector<b2Fixture *> Box2DSensor::b2_fixtures()
{
    return b2_fixtures_;
}

std::vector<b2Fixture const *> Box2DSensor::b2_fixtures() const
{
    // Unfortunately, C++ does not support covariant template specialization;
    // e.g. replacing "T" with "const T". We need to convert manually.
    std::vector<b2Fixture const *> fixtures;
    fixtures.reserve(b2_fixtures_.size());

    BOOST_FOREACH (b2Fixture *const b2_fixture, b2_fixtures_) {
        fixtures.push_back(b2_fixture);
    }
    return fixtures;
}

}
