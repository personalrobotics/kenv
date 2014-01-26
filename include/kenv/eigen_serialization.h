#ifndef EIGEN_SERIALIZATION_H_
#define EIGEN_SERIALIZATION_H_
#include <boost/foreach.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/split_free.hpp>
#include <Eigen/Dense>
#include <Eigen/Sparse>

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
 * SparseMatrix
 * Inspired by: https://gist.github.com/mtao/5798888
 */
template <class Scalar>
class EigenTriplet {
public:
    EigenTriplet() {}
    EigenTriplet(int row, int col, Scalar const &value)
        : row_(row), col_(col), value_(value) {}

    int row() const { return row_; }
    int col() const { return col_; }
    Scalar value() const { return value_; };

private: 
    int row_, col_;
    Scalar value_;

    template <class Archive>
    void serialize(Archive &archive, unsigned int const version)
    {
        archive & row_ & col_ & value_;
    }

    friend class boost::serialization::access;
};

template <class Archive, typename _Scalar, int _Options, typename _Index>
inline void save(Archive &archive, Eigen::SparseMatrix<_Scalar, _Options, _Index> const &m,
                 unsigned int const version)
{
    //typedef typename Eigen::Triplet<_Scalar> Triplet;
    typedef EigenTriplet<_Scalar> Triplet;
    typedef typename Eigen::SparseMatrix<_Scalar, _Options, _Index>::InnerIterator InnerIterator;

    int innerSize = m.innerSize();
    int outerSize = m.outerSize();

    std::vector<Triplet> triplets;

    for (int i=0; i < outerSize; ++i) {
        for (InnerIterator it(m,i); it; ++it) {
            triplets.push_back(Triplet(it.row(), it.col(), it.value()));
        }
    }

    archive & innerSize & outerSize & triplets;
}

template <class Archive, typename _Scalar, int _Options, typename _Index>
inline void load(Archive &archive, Eigen::SparseMatrix<_Scalar, _Options, _Index> &m,
                 unsigned int const version) {
    int innerSize;
    int outerSize;
    archive & innerSize & outerSize;

    int rows = m.IsRowMajor ? outerSize : innerSize;
    int cols = m.IsRowMajor ? innerSize : outerSize;
    m.resize(rows,cols);

    //typedef typename Eigen::Triplet<_Scalar> Triplet;
    typedef EigenTriplet<_Scalar> Triplet;
    std::vector<Triplet> triplets;
    archive & triplets;

    //m.setFromTriplets(triplets.begin(), triplets.end());
    m.setZero();
    BOOST_FOREACH (Triplet const &triplet, triplets) {
        m.insert(triplet.row(), triplet.col()) = triplet.value();
    }
    m.finalize();
}

template <class Archive, typename _Scalar, int _Options, typename _Index>
void serialize(Archive &archive, Eigen::SparseMatrix<_Scalar,_Options,_Index> &m,
               unsigned int const version)
{
    boost::serialization::split_free(archive, m, version);
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
