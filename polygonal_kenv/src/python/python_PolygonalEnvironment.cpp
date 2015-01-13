#include "PolygonalEnvironment.h"
#include <boost/python.hpp>
#include <boost/version.hpp>
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

#if 0
struct geos_from_python
{
    geos_from_python()
    {
        boost::python::converter::registry::push_back(
            &convertible, &construct, boost::python::type_id<GeometryPtr>()
        );
#if 0
        boost::python::converter::registry::push_back(
            &convertible, &construct,
            boost::python::type_id<geos::geom::Geometry>()
        );
        boost::python::converter::registry::push_back(
            &convertible, &construct,
            boost::python::type_id<boost::shared_ptr<geos::geom::Geometry const> >()
        );
#endif
        std::cout << "REGISTERING" << std::endl;
    }

    static void *convertible(PyObject *obj_ptr)
    {
        std::cout << "CHECKING CONVERTIBLE" << std::endl;
        return NULL;
    }

    static void construct(PyObject *py_obj_ptr,
                          boost::python::converter::rvalue_from_python_stage1_data *data)
    {
        std::cout << "TRYING TO CONVERT" << std::endl;
        boost::python::object const ns = boost::python::import("shapely.wkb");
        boost::python::object const dumps = ns.attr("dumps");
#if 0
        boost::python::object const py_obj(boost::python::handle<>(py_obj_ptr));
        boost::python::object const py_wkb = dumps(py_obj);
        boost::python::extract<std::string> extract_string(py_wkb);
        if (!extract_string.check()) {
            throw std::runtime_error("Unable to extract geometry's WKB.");
        }

        std::cout << "Loading from WKB." << std::endl;
        std::stringstream ss(extract_string());
        geos::io::WKBReader reader;
        geos::geom::Geometry *geom = reader.read(ss);
        // TODO: Wrap in a shared_ptr.
#endif
    }
};
#endif

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
    // to_python converters are broken for abstract classes, like
    // geos::Geometry in new versions of Boost.Python. Previously, this worked
    // since the object is passed by reference.
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
