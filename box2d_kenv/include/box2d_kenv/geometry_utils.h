#ifndef GEOMETRY_UTILS_H_
#define GEOMETRY_UTILS_H_
#include <list>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/MP_Float.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/partition_2.h>
#include <CGAL/Quotient.h>
#include <geos/geom/Polygon.h>

namespace box2d_kenv {

namespace util {

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Partition_traits_2<Kernel>                    Traits;
typedef Traits::Point_2                                     Point_2;
typedef Traits::Polygon_2                                   Polygon_2;
typedef Polygon_2::Vertex_iterator                          Vertex_iterator;
typedef std::list<Polygon_2>                                Polygon_list;

Polygon_2 ConvertPolygonGEOStoCGAL(geos::geom::Polygon const &geom,
                                   double scale, double weld_distance);

Polygon_list DecomposeCGALPolygon(Polygon_2 const &polygon);

std::pair<Polygon_2, Polygon_2> SplitCGALPolygon(Polygon_2 const &polygon);

size_t RecursivelySplitCGALPolygon(Polygon_2 const &polygon, size_t max_vertices,
                                   Polygon_list *output_polygons);

b2PolygonShape ConvertPolygonCGALtoBox2D(Polygon_2 const &polygon);

}

}

#endif
