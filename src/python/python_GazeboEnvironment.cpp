#include <gz_kenv/gz_kenv.h>
#include <boost/python.hpp>
#include <geos/geom/Geometry.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/io/WKBWriter.h>
#include <geos/io/WKBReader.h>
#include <kenv/python_pickle_helpers.h>
#include "gazebo/physics/physics.hh"


using namespace boost::python;
using namespace kenv;

void python_GazeboEnvironment()
{
    class_<GazeboEnvironment, boost::noncopyable, bases<kenv::Environment>,
           GazeboEnvironment::Ptr>("GazeboEnvironment",
            boost::python::init<gazebo::physics::WorldPtr>())
         .def_pickle(kenv_util::empty_pickle_wrapper<GazeboEnvironment>())
         .def("getGazeboEnvironment", &GazeboEnvironment::getGazeboEnvironment)
         ;
   

}
