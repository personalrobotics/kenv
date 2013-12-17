#define BOOST_PYTHON_STATIC_LIB
#include <boost/python.hpp>
#include "Environment.h"

BOOST_PYTHON_MODULE(libkenv_ext)
{
    boost::python::class_<kenv::Contact>("Contact")
        .def_readwrite("position", &kenv::Contact::position)
        .def_readwrite("normal", &kenv::Contact::normal);

    boost::python::class_<kenv::Link, boost::noncopyable>("Link", boost::python::no_init)
        .def("getObject", &kenv::Link::getObject)
        .def("getName", &kenv::Link::getName)
        .def("getTransform", &kenv::Link::getTransform)
        .def("enable", &kenv::Link::enable)
        .def("computeLocalAABB", &kenv::Link::computeLocalAABB);

    boost::python::class_<kenv::Object, boost::noncopyable>("Object", boost::python::no_init)
        .def("getEnvironment", &kenv::Object::getEnvironment)
        .def("getName", &kenv::Object::getName)
        .def("getType", &kenv::Object::getType)
        .def("getKinematicsGeometryHash", &kenv::Object::getKinematicsGeometryHash)
        .def("enable", &kenv::Object::enable)
        .def("setVisible", &kenv::Object::setVisible)
        .def("getAABB", &kenv::Object::getAABB)
        .def("getLinks", &kenv::Object::getLinks)
        .def("getLink", &kenv::Object::getLink)
        .def("getTransform", &kenv::Object::getTransform)
        .def("setTransform", &kenv::Object::setTransform)
        .def("getDOFValues", &kenv::Object::getDOFValues)
        .def("setDOFValues", &kenv::Object::setDOFValues)
        .def("setColor", &kenv::Object::setColor)
        .def("setTransparency", &kenv::Object::setTransparency);

    boost::python::class_<kenv::Environment, boost::noncopyable>("Environment", boost::python::no_init)
        .def("getObject", &kenv::Environment::getObject)
        .def("createObject", &kenv::Environment::createObject)
        .def("remove", &kenv::Environment::remove)
        .def("drawLine", &kenv::Environment::drawLine)
        .def("drawLineStrip", &kenv::Environment::drawLineStrip)
        .def("drawLineList", &kenv::Environment::drawLineList)
        .def("drawArrow", &kenv::Environment::drawArrow)
        .def("drawPoints", &kenv::Environment::drawPoints)
        .def("drawPlane", &kenv::Environment::drawPlane);
}
