#include <boost/python.hpp>
#include <geos/geom/Geometry.h>
#include <geos/io/WKBWriter.h>
#include <geos/io/WKTWriter.h>
#include "PolygonalEnvironment.h"

#include <geos/geom/GeometryFactory.h>

using namespace boost::python;
using namespace kenv;

typedef boost::shared_ptr<geos::geom::Geometry> GeometryPtr;

std::string toWKB(geos::geom::Geometry const &geom)
{
    std::ostringstream ss;
    static geos::io::WKBWriter writer;
    writer.write(geom, ss);
    return ss.str();
}

std::string toWKT(geos::geom::Geometry const &geom)
{
    static geos::io::WKTWriter writer;
    return writer.write(&geom);
}

void python_PolygonalEnvironment()
{
    register_ptr_to_python<boost::shared_ptr<geos::geom::Geometry const> >(); 

    class_<geos::geom::Geometry, GeometryPtr, boost::noncopyable>
           ("GEOSGeometry", no_init)
        .add_property("wkb", &toWKB)
        .add_property("wkt", &toWKT)
        ;

    class_<PolygonalEnvironment, boost::noncopyable,
           bases<Environment>, PolygonalEnvironment::Ptr>
           ("PolygonalEnvironment")
        ;

    class_<PolygonalObject, boost::noncopyable,
           bases<Object>, PolygonalObject::Ptr>
           ("PolygonalObject", no_init)
        .add_property("geometry", &PolygonalObject::getGeometry)
        ;
}
