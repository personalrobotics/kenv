#include <geos/geom/Coordinate.h>
#include "AffineTransformFilter.h"

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
