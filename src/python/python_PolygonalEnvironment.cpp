#include <boost/python.hpp>
#include "PolygonalEnvironment.h"

using namespace boost::python;
using namespace kenv;

void python_PolygonalEnvironment()
{
    class_<PolygonalEnvironment, boost::noncopyable,
           bases<Environment>, PolygonalEnvironment::Ptr>
             ("PolygonalEnvironment");
}
