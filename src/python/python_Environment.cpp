#include <boost/foreach.hpp>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include "Environment.h"

using namespace boost::python;
using namespace kenv;

static boost::python::list Object_getLinks(Object const *instance)
{
    BOOST_ASSERT(instance);
    std::vector<kenv::Link::Ptr> links = instance->getLinks();

    boost::python::list py_links;
    BOOST_FOREACH (kenv::Link::Ptr link, links) {
        py_links.append(link);
    }
    return py_links;
}

static bool Object_checkCollision(kenv::Object const *instance,
                                  kenv::Object::Ptr other)
{
    return instance->checkCollision(other);
}

void python_Environment()
{
    class_<Contact>("Contact")
        .def_readwrite("position", &Contact::position)
        .def_readwrite("normal", &Contact::normal)
        ;

    class_<Link, boost::noncopyable, Link::Ptr>("Link", no_init)
        .add_property("object", &Link::getObject)
        .add_property("name", &Link::getName)
        .add_property("pose", &Link::getTransform)
        .add_property("computeLocalAABB", &Link::computeLocalAABB)
        .def("enable", &Link::enable)
        ;

    class_<Object, boost::noncopyable, Object::Ptr>("Object", no_init)
        .def("getEnvironment", &Object::getEnvironment)
        .def("getName", &Object::getName)
        .def("getType", &Object::getType)
        .def("getKinematicsGeometryHash", &Object::getKinematicsGeometryHash)
        .def("enable", &Object::enable)
        .def("setVisible", &Object::setVisible)
        .def("getAABB", &Object::getAABB)
        .def("getLinks", &Object_getLinks)
        .def("getLink", &Object::getLink)
        .def("getTransform", &Object::getTransform)
        .def("setTransform", &Object::setTransform)
        .def("getDOFValues", &Object::getDOFValues)
        .def("setDOFValues", &Object::setDOFValues)
        .def("setColor", &Object::setColor)
        .def("setTransparency", &Object::setTransparency)
        .def("checkCollision", &Object_checkCollision)
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

    class_<std::vector<Link::Ptr> >("LinkVector")
        .def(vector_indexing_suite<std::vector<Link::Ptr> >())
        ;
}
