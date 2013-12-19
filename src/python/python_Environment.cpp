#include <boost/python.hpp>
#include "Environment.h"

using namespace boost::python;
using namespace kenv;

void python_Environment()
{
    class_<Contact>("Contact")
        .def_readwrite("position", &Contact::position)
        .def_readwrite("normal", &Contact::normal)
        ;

    class_<Link, boost::noncopyable, Link::Ptr>("Link", no_init)
        .def("getObject", &Link::getObject)
        .def("getName", &Link::getName)
        .def("getTransform", &Link::getTransform)
        .def("enable", &Link::enable)
        .def("computeLocalAABB", &Link::computeLocalAABB)
        ;

    class_<Object, boost::noncopyable, Object::Ptr>("Object", no_init)
        .def("getEnvironment", &Object::getEnvironment)
        .def("getName", &Object::getName)
        .def("getType", &Object::getType)
        .def("getKinematicsGeometryHash", &Object::getKinematicsGeometryHash)
        .def("enable", &Object::enable)
        .def("setVisible", &Object::setVisible)
        .def("getAABB", &Object::getAABB)
        .def("getLinks", &Object::getLinks)
        .def("getLink", &Object::getLink)
        .def("getTransform", &Object::getTransform)
        .def("setTransform", &Object::setTransform)
        .def("getDOFValues", &Object::getDOFValues)
        .def("setDOFValues", &Object::setDOFValues)
        .def("setColor", &Object::setColor)
        .def("setTransparency", &Object::setTransparency)
        ;

    class_<Environment, boost::noncopyable, Environment::Ptr>("Environment", no_init)
        .def("getObject", &Environment::getObject)
        .def("createObject", &Environment::createObject)
        .def("remove", &Environment::remove)
        .def("drawLine", &Environment::drawLine)
        .def("drawLineStrip", &Environment::drawLineStrip)
        .def("drawLineList", &Environment::drawLineList)
        .def("drawArrow", &Environment::drawArrow)
        .def("drawPoints", &Environment::drawPoints)
        .def("drawPlane", &Environment::drawPlane)
        ;
}
