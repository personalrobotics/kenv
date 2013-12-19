#include <boost/python.hpp>
#include "CollisionChecker.h"

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
        ;
}
