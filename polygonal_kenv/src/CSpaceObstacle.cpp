#define CGAL_DISABLE_ROUNDING_MATH_CHECK
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <geos/geom/CoordinateSequenceFactory.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/LinearRing.h>
#include <geos/geom/Polygon.h>
#include <CGAL/MP_Float.h>
#include <CGAL/Quotient.h>
#include <CGAL/Cartesian.h>
#include <CGAL/minkowski_sum_2.h>
#include "CSpaceObstacle.h"

template <class Kernel>
static CGAL::Polygon_2<Kernel> GEOSToCGAL_Polygon(geos::geom::Polygon const *geos_polygon)
{
    CGAL::Polygon_2<Kernel> cgal_polygon;
    if (geos_polygon->isEmpty()) {
        return cgal_polygon;
    }

    // Omit the last point to avoid closing the polygon. This is necessary
    // because GEOS stores closed polygons, while CGAL stores open polygons.
    boost::shared_ptr<geos::geom::CoordinateSequence> coords(
        geos_polygon->getCoordinates()
    );

    for (size_t i = 0; i < coords->size() - 1; ++i) {
        geos::geom::Coordinate const &geos_point = (*coords)[i];
        typename Kernel::Point_2 const cgal_point(geos_point.x, geos_point.y);
        cgal_polygon.push_back(cgal_point);
    }
    return cgal_polygon;
}

template <class Kernel>
static CGAL::Polygon_2<Kernel> GEOSToCGAL_LineString(geos::geom::LineString const *geos_linestring)
{
    CGAL::Polygon_2<Kernel> cgal_polygon;
    if (geos_linestring->isEmpty()) {
        return cgal_polygon;
    }

    boost::shared_ptr<geos::geom::CoordinateSequence> coords(
        geos_linestring->getCoordinates()
    );

    for (size_t i = 0; i < coords->size(); ++i) {
        geos::geom::Coordinate const &geos_point = (*coords)[i];
        typename Kernel::Point_2 const cgal_point(geos_point.x, geos_point.y);
        cgal_polygon.push_back(cgal_point);
    }
    return cgal_polygon;
}

template <class Kernel>
static CGAL::Polygon_2<Kernel> GEOSToCGAL(geos::geom::Geometry const *geos_geometry)
{
    switch (geos_geometry->getGeometryTypeId()) {
    case geos::geom::GEOS_POLYGON:
        return GEOSToCGAL_Polygon<Kernel>(dynamic_cast<geos::geom::Polygon const *>(geos_geometry));

    case geos::geom::GEOS_LINESTRING:
        return GEOSToCGAL_LineString<Kernel>(dynamic_cast<geos::geom::LineString const *>(geos_geometry));

    default:
        throw std::runtime_error(boost::str(
            boost::format("Unable to convert GEOS geometry to CGAL; unsupported type \"%s\".")
                % geos_geometry->getGeometryType()));
    }
}

template <class Kernel>
static geos::geom::Polygon *CGALToGEOS(CGAL::Polygon_2<Kernel> const &cgal_polygon)
{
    geos::geom::GeometryFactory const *geom_factory = geos::geom::GeometryFactory::getDefaultInstance();
    geos::geom::CoordinateSequenceFactory const *cs_factory = geom_factory->getCoordinateSequenceFactory();

    geos::geom::CoordinateSequence *coordinates = cs_factory->create(0);
    BOOST_FOREACH (typename Kernel::Point_2 const &cgal_point, cgal_polygon.container()) {
        geos::geom::Coordinate geos_point;
        geos_point.x = CGAL::to_double(cgal_point.x());
        geos_point.y = CGAL::to_double(cgal_point.y());
        coordinates->add(geos_point);
    }

    // Manually close the polygon. This is necessary because GEOS stores closed
    // polygons, while CGAL stores open polygons.
    coordinates->add(coordinates->front());

    geos::geom::LinearRing *linear_ring = geom_factory->createLinearRing(coordinates);
    std::vector<geos::geom::Geometry *> *holes = new std::vector<geos::geom::Geometry *>;
    return geom_factory->createPolygon(linear_ring, holes);
}

geos::geom::Geometry *ComputeCSpaceObstacle(geos::geom::Geometry const *robot,
                                            geos::geom::Geometry const *obstacle)
{
    typedef CGAL::Quotient<CGAL::MP_Float>              Number_type;
    typedef CGAL::Cartesian<Number_type>                Kernel;
    typedef Kernel::Point_2                             Point_2;
    typedef CGAL::Polygon_2<Kernel>                     Polygon_2;
    typedef CGAL::Polygon_with_holes_2<Kernel>          Polygon_with_holes_2;

    Polygon_2 const cgal_robot = GEOSToCGAL<Kernel>(robot);
    Polygon_2 const cgal_obstacle = GEOSToCGAL<Kernel>(obstacle);

    // Use the Minkowski sum to compute the C-space obstacle.
    Polygon_with_holes_2 minkowski_sum = minkowski_sum_2(cgal_robot, cgal_obstacle);
    if (minkowski_sum.number_of_holes() > 0) {
        throw std::runtime_error("C-space obstacle has holes.");
    }
    return CGALToGEOS(minkowski_sum.outer_boundary());
}
