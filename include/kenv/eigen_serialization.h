#ifndef EIGEN_SERIALIZATION_H_
#define EIGEN_SERIALIZATION_H_
#include <Eigen/Dense>

namespace boost {

/*
 * Eigen::Matrix
 * Source: http://stackoverflow.com/questions/12580579/how-to-use-boostserialization-to-save-eigenmatrix
 */
template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline void serialize(Archive &ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> &t, 
                      unsigned int const version) 
{
    int rows = t.rows();
    int cols = t.cols();

    ar & rows;
    ar & cols;

    if (rows * cols != t.size()) {
        t.resize(rows, cols);
    }

    for (int i = 0; i < t.size(); ++i) {
        ar & t.data()[i];
    }
}

/*
 * Eigen::Array
 * Source: http://stackoverflow.com/questions/12580579/how-to-use-boostserialization-to-save-eigenmatrix
 */
template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline void serialize(Archive &ar, Eigen::Array<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> &t, 
                      unsigned int const version) 
{
    int rows = t.rows();
    int cols = t.cols();

    ar & rows;
    ar & cols;

    if (rows * cols != t.size()) {
        t.resize(rows, cols);
    }

    for (int i = 0; i < t.size(); ++i) {
        ar & t.data()[i];
    }
}

/*
 * Eigen::Transform (e.g. Eigen::Affine3d)
 */
template <class Archive, typename _Scalar, int _Dim, int _Mode, int _Options>
inline void serialize(Archive &ar, Eigen::Transform<_Scalar, _Dim, _Mode, _Options> &transform,
                      unsigned int const version) 
{
    ar & transform.matrix();
}

/*
 * Eigen::AlignedBox (AABBs)
 */
template <class Archive, typename _Scalar, int _AmbientDim> 
inline void serialize(Archive &ar, Eigen::AlignedBox<_Scalar, _AmbientDim> &aabb,
                      unsigned int const version) 
{
    ar & aabb.min() & aabb.max();
}

}

#endif
