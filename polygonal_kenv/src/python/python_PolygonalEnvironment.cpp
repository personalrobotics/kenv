#include "PolygonalEnvironment.h"
#include <boost/python.hpp>
#include <geos/geom/Geometry.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/io/WKBWriter.h>
#include <geos/io/WKBReader.h>
#include <kenv/python_pickle_helpers.h>

using namespace boost::python;
using namespace kenv;

typedef boost::shared_ptr<geos::geom::Geometry> GeometryPtr;
typedef boost::shared_ptr<geos::geom::Geometry const> GeometryConstPtr;

struct geos_to_python {
    static PyObject *convert(geos::geom::Geometry const &geom)
    {
        // Dump the geometry to WKB.
        geos::io::WKBWriter writer;
        std::ostringstream ss;
        writer.write(geom, ss);

        // Load a Shapely object from WKB.
        boost::python::object const ns = boost::python::import("shapely.wkb");
        boost::python::object const loads = ns.attr("loads");
        boost::python::object const py_geom = loads(ss.str());
        return boost::python::incref(boost::python::object(py_geom).ptr());
    }
};

// This is a hack to work around Boost.Python's poor support for const.
template <class T, class Delegate>
struct ptr_to_python {
    static PyObject *convert(boost::shared_ptr<T> const &ptr)
    {
        return Delegate().convert(*ptr);
    }
};

template <class T, class Delegate>
struct geos_ptr_to_python {
    static PyObject *convert(boost::shared_ptr<T> const &ptr)
    {
        return geos_to_python::convert(*ptr);
    }
};

template <class T, class Delegate>
void ptr_to_python_converter()
{
#if BOOST_VERSION > 104601
    to_python_converter<boost::shared_ptr<T>, geos_ptr_to_python<T, Delegate> >();
    to_python_converter<boost::shared_ptr<T const>, geos_ptr_to_python<T const, Delegate> >();
#else
    to_python_converter<T, Delegate>();
    to_python_converter<boost::shared_ptr<T>, ptr_to_python<T, Delegate> >();
    to_python_converter<boost::shared_ptr<T const>, ptr_to_python<T const, Delegate> >();
#endif
}

void python_PolygonalEnvironment()
{
    ptr_to_python_converter<geos::geom::Geometry, geos_to_python>();

    class_<PolygonalEnvironment, boost::noncopyable, bases<Environment>,
           PolygonalEnvironment::Ptr>("PolygonalEnvironment")
        .def_pickle(kenv_util::empty_pickle_wrapper<PolygonalEnvironment>())
        ;

    class_<PolygonalObject, boost::noncopyable, bases<Object>,
           PolygonalObject::Ptr>("PolygonalObject", no_init)
        .add_property("geometry", &PolygonalObject::getGeometry)
        ;

    class_<PolygonalLink_ext, boost::noncopyable, bases<Link>,
           boost::shared_ptr<PolygonalLink_ext> >("PolygonalLink", no_init)
        .add_property("geometry", &PolygonalLink_ext::getGeometry)
        ;
}
