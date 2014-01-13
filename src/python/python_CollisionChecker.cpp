#include <boost/python.hpp>
#include "CollisionChecker.h"
#include "python_pickle_helpers.h"

using namespace boost::python;
using namespace kenv;

void python_CollisionChecker()
{
    class_<CollisionChecker, boost::noncopyable,
           CollisionChecker::Ptr>("CollisionChecker", no_init)
        .def("CheckCollision", &CollisionChecker::checkCollision)
        ;

    class_<DefaultCollisionChecker, bases<CollisionChecker>,
           DefaultCollisionChecker::Ptr>("DefaultCollisionChecker")
        .def_pickle(util::empty_pickle_wrapper<CollisionChecker>())
        ;
}
