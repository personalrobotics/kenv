#include "PolygonalEnvironment.h"
#include <boost/python.hpp>
#include <geos/geom/Geometry.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/io/WKBWriter.h>
#include <geos/io/WKTWriter.h>
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
struct const_ptr_to_python {
    static PyObject *convert(boost::shared_ptr<T const> const &const_ptr)
    {
        return Delegate().convert(*const_ptr);
    }
};

void python_PolygonalEnvironment()
{
    to_python_converter<geos::geom::Geometry, geos_to_python>();
    register_ptr_to_python<GeometryPtr>();
    to_python_converter<GeometryConstPtr,
                        const_ptr_to_python<geos::geom::Geometry, geos_to_python> >();

    class_<PolygonalEnvironment, boost::noncopyable, bases<Environment>,
           PolygonalEnvironment::Ptr>("PolygonalEnvironment")
        .def_pickle(util::empty_pickle_wrapper<PolygonalEnvironment>())
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
