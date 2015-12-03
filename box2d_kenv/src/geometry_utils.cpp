#include <boost/format.hpp>
#include <geos/geom/Coordinate.h>
#include <geos/geom/CoordinateSequence.h>
#include "geometry_utils.h"

namespace box2d_kenv {

namespace util {

Polygon_2 ConvertPolygonGEOStoCGAL(geos::geom::Polygon const &geom,
                                   double scale, double weld_distance)
{
    //double const distance_threshold = 0.5 * b2_linearSlop / scale;
    boost::shared_ptr<geos::geom::CoordinateSequence> coords(geom.getCoordinates());
    if (coords->size() < 4) {
        throw std::runtime_error(boost::str(
            boost::format("Invalid geometry. A polygon must contain three or"
                          "more points; got %d.") % (coords->size() - 1)));
    }

    BOOST_ASSERT(scale > 0.);
    BOOST_ASSERT(weld_distance > 0.);

    // The polygon is a closed contour, so the last point is a neighbor of the
    // first point.
    geos::geom::Coordinate const first_point = (*coords)[0];
    geos::geom::Coordinate const last_point = (*coords)[coords->size() - 1];
    BOOST_ASSERT(first_point.x == last_point.x
              && first_point.y == last_point.y);

    // We intentionally omit the last point to avoid closing the polygon. This
    // is necessary because GEOS stores closed polygons, while CGAL stores open
    // polygons.
    geos::geom::Coordinate prev_point = (*coords)[coords->size() - 2];
    std::vector<Point_2> vertices; 

    for (size_t i = 0; i < coords->size() - 1; ++i) {
        geos::geom::Coordinate const &curr_point = (*coords)[i];

        // Skip sequential points that are too close together. Box2D requires
        // that no two points on the same polygon be within the "slop radius"
        // of each other.
        // TODO: This is called "vertex welding" and seems to occur by default
        // inside b2Polygon::Set inBo2D.
        if (curr_point.distance(prev_point) > weld_distance) {
            Point_2 const cgal_point(scale * curr_point.x, scale * curr_point.y);
            vertices.push_back(cgal_point);
            prev_point = curr_point;
        }
    }

    Polygon_2 cgal_polygon(vertices.begin(), vertices.end());

    // Reverse the polygon if it is clockwise oriented.
    if (cgal_polygon.is_clockwise_oriented()) {
        cgal_polygon = Polygon_2(vertices.rbegin(), vertices.rend());
    }
    return cgal_polygon;
}

b2PolygonShape ConvertPolygonCGALtoBox2D(Polygon_2 const &polygon)
{
    if (polygon.is_clockwise_oriented()) {
        throw std::runtime_error("Box2D only supports counter-clockwise polygons.");
    }

    std::vector<b2Vec2> b2_vertices;
    b2_vertices.reserve(polygon.size());

    for (Polygon_2::Vertex_const_iterator it = polygon.vertices_begin();
         it != polygon.vertices_end();
         ++it)
    {
        b2_vertices.push_back(b2Vec2(it->x(), it->y()));
    }

    // The Set() function copies vertex data into an internal buffer.
    b2PolygonShape shape;
    shape.Set(b2_vertices.data(), b2_vertices.size());
    return shape;
}

Polygon_list DecomposeCGALPolygon(Polygon_2 const &polygon)
{
    // The approx_convex_partition_2 algorithm expects the polygon to be in
    // counterclockwise order. We'll flip the order of the vertices if it's in
    // clockwise order.
    Polygon_list convex_polygons;
    CGAL::approx_convex_partition_2(
        polygon.vertices_begin(), polygon.vertices_end(),
        std::back_inserter(convex_polygons)
    );

    bool const is_valid = CGAL::convex_partition_is_valid_2(
        polygon.vertices_begin(), polygon.vertices_end(),
        convex_polygons.begin(), convex_polygons.end()
    );
    if (is_valid) {
        return convex_polygons;
    } else {
        throw std::runtime_error("Convex polygon decomposition is invalid.");
    }
}

std::pair<Polygon_2, Polygon_2> SplitCGALPolygon(Polygon_2 const &polygon)
{
    BOOST_ASSERT(polygon.size() > 3);

    std::pair<Polygon_2, Polygon_2> split_polygons;
    Polygon_2 &split_polygon1 = split_polygons.first;
    Polygon_2 &split_polygon2 = split_polygons.second;

    // Create the first polygon, containing vertices [ 0, half_index ].
    Polygon_2::Vertex_iterator vertex_it = polygon.vertices_begin();
    size_t const half_index = polygon.size() / 2;

    for (size_t ivertex = 0; ivertex <= half_index; ++ivertex) {
        split_polygon1.push_back(*vertex_it);
        ++vertex_it;
    }

    // Create the second polygon, containing vertex 0 and the vertices [
    // half_index, n - 1 ]. Note that we have to include vertex 0 and
    // half_index to avoid introducing a gap between the two halves.
    split_polygon2.push_back(*polygon.vertices_begin());

    --vertex_it;
    for (size_t ivertex = half_index; ivertex < polygon.size(); ++ivertex) {
        split_polygon2.push_back(*vertex_it);
        ++vertex_it;
    }

    BOOST_ASSERT(3 <= split_polygon1.size() && split_polygon1.size() < polygon.size());
    BOOST_ASSERT(3 <= split_polygon2.size() && split_polygon2.size() < polygon.size());

    return split_polygons;
}

size_t RecursivelySplitCGALPolygon(Polygon_2 const &polygon, size_t max_vertices,
                                          Polygon_list *output_polygons)
{
    BOOST_ASSERT(max_vertices >= 3);
    BOOST_ASSERT(output_polygons);

    size_t num_added = 0;
    Polygon_list invalid_polygons, valid_polygons;
    invalid_polygons.push_back(polygon);

    while (!invalid_polygons.empty()) {
        Polygon_2 const &candidate_polygon = invalid_polygons.front();

        // The candidate polygon is too big. Split it in half and try again.
        if (candidate_polygon.size() > max_vertices) {
            std::pair<Polygon_2, Polygon_2> const split_polygon
                = SplitCGALPolygon(candidate_polygon);
            invalid_polygons.push_back(split_polygon.first);
            invalid_polygons.push_back(split_polygon.second);
        } else {
            output_polygons->push_back(candidate_polygon);
            num_added++;
        }

        invalid_polygons.pop_front();
    }

    return num_added;
}

/*
 * AffineTransformFilter
 */
AffineTransformFilter::AffineTransformFilter(Eigen::Affine2d const &transform)
    : transform_(transform)
{
}

void AffineTransformFilter::filter_rw(geos::geom::Coordinate *coord) const
{
    Eigen::Vector2d const eigen_before(coord->x, coord->y);
    Eigen::Vector2d const eigen_after = transform_ * eigen_before;
    coord->x = eigen_after[0];
    coord->y = eigen_after[1];
}

}
}
