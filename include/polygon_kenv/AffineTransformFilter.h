#ifndef AFFINETRANSFORMFILTER_H_
#define AFFINETRANSFORMFILTER_H_

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <geos/geom/CoordinateFilter.h>

class AffineTransformFilter : public geos::geom::CoordinateFilter {
public:
    typedef boost::shared_ptr<AffineTransformFilter> Ptr;
    typedef boost::shared_ptr<AffineTransformFilter const> ConstPtr;

    AffineTransformFilter(Eigen::Affine2d const &transform);
    virtual void filter_rw(geos::geom::Coordinate *coord) const;

private:
    Eigen::Affine2d const transform_;
};

#endif
