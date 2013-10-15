#ifndef EIGEN_HASH_H_
#define EIGEN_HASH_H_

#include <Eigen/Dense>
#include <boost/functional/hash.hpp>

template <class Derived>
std::size_t hash_value(Eigen::MatrixBase<Derived> const &matrix)
{
    std::size_t seed = 0;

    for (size_t i = 0; i < matrix.rows(); ++i)
    for (size_t j = 0; j < matrix.cols(); ++j) {
        boost::hash_combine(seed, boost::hash_value(matrix(i, j)));
    }

    return seed;
}

#endif
