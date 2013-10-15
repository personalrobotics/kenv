#define CGAL_DISABLE_ROUNDING_MATH_CHECK
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
static CGAL::Polygon_2<Kernel> GEOSToCGAL(geos::geom::Polygon const *geos_polygon)
{
    CGAL::Polygon_2<Kernel> cgal_polygon;
    if (geos_polygon->isEmpty()) {
        return cgal_polygon;
    }

    // Omit the last point to avoid closing the polygon. This is necessary
    // because GEOS stores closed polygons, while CGAL stores open polygons.
    geos::geom::CoordinateSequence *coords = geos_polygon->getCoordinates();
    for (size_t i = 0; i < coords->size() - 1; ++i) {
        geos::geom::Coordinate const &geos_point = (*coords)[i];
        typename Kernel::Point_2 const cgal_point(geos_point.x, geos_point.y);
        cgal_polygon.push_back(cgal_point);
    }

    delete coords;
    return cgal_polygon;
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

    // Extract the objects' polygonal geometry.
    if (robot->getGeometryTypeId() != geos::geom::GEOS_POLYGON) {
        throw std::runtime_error("Robot is not a polygon.");
    } else if (obstacle->getGeometryTypeId() != geos::geom::GEOS_POLYGON) {
        throw std::runtime_error("Obstacle is not a polygon.");
    }
    geos::geom::Polygon const *geos_robot = dynamic_cast<geos::geom::Polygon const *>(robot);
    geos::geom::Polygon const *geos_obstacle = dynamic_cast<geos::geom::Polygon const *>(obstacle);
    BOOST_ASSERT(geos_robot && geos_obstacle);

    // Convert from GEOS to CGAL.
    Polygon_2 cgal_robot = GEOSToCGAL<Kernel>(geos_robot);
    Polygon_2 cgal_obstacle = GEOSToCGAL<Kernel>(geos_obstacle);

    // Use the Minkowski sum to compute the C-space obstacle.
    Polygon_with_holes_2 minkowski_sum = minkowski_sum_2(cgal_robot, cgal_obstacle);
    if (minkowski_sum.number_of_holes() > 0) {
        throw std::runtime_error("C-space obstacle has holes.");
    }
    return CGALToGEOS(minkowski_sum.outer_boundary());
}
