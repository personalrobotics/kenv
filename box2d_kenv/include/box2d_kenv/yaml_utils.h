#ifndef YAML_UTILS_H_
#define YAML_UTILS_H_
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

namespace box2d_kenv {
namespace util {

template <class Derived>
inline void deserialize(YAML::Node const &node, Eigen::MatrixBase<Derived> &matrix)
{
    if (node.Type() != YAML::NodeType::Sequence) {
        throw std::runtime_error("Matrix or vector must be a sequence.");
    }

    size_t const rows = node.size();
    if (node.Tag() == "!Vector") {
        matrix.resize(rows, 1);

        for (size_t i = 0; i < rows; ++i) {
#ifdef YAMLCPP_NEWAPI
            matrix(i, 0) = node[i].as<typename Derived::Scalar>();
#else
            matrix(i, 0) = node[i].to<typename Derived::Scalar>();
#endif
        }
    } else if (node.Tag() == "!Matrix") {
        size_t const cols = node[0].size();
        matrix.resize(rows, cols);

        for (size_t r = 0; r < node.size(); ++r) {
            if (node[r].Type() != YAML::NodeType::Sequence) {
                throw std::runtime_error(boost::str(
                    boost::format("Row %d of the matrix must be a sequence.") % r));
            } else if (node[r].size() != cols) {
                throw std::runtime_error(boost::str(
                    boost::format("Expected row %d to have %d columns; got %d.")
                        % r % cols % node[r].size()));
            }

            for (size_t c = 0; c < cols; ++c) {
#ifdef YAMLCPP_NEWAPI
                matrix(r, c) = node[r][c].as<typename Derived::Scalar>();
#else
                matrix(r, c) = node[r][c].to<typename Derived::Scalar>();
#endif
            }
        }
    } else {
        throw std::runtime_error(boost::str(
            boost::format("Unknown type of matrix '%s'.") % node.Tag()));
    }
}

template <class Derived, int Dim, int Mode, int _Options>
inline void deserialize(YAML::Node const &node,
                        Eigen::Transform<Derived, Dim, Mode, _Options> &pose)
{
  deserialize(node, pose.matrix());
}

// TODO: I should really specialize the YAML::convert class.
template <class Derived>
inline YAML::Node serialize(Eigen::MatrixBase<Derived> const &matrix)
{
    YAML::Node node;

    if (Eigen::MatrixBase<Derived>::IsVectorAtCompileTime) {
        node.SetTag("Vector");

        for (int i = 0; i < matrix.size(); ++i) {
            node.push_back(matrix(i, 0));
        }
    } else {
        node.SetTag("Matrix");

        for (int r = 0; r < matrix.rows(); ++r) {
            YAML::Node subnode;

            for (int c = 0; c < matrix.cols(); ++c) {
                subnode.push_back(matrix(r, c));
            }

            node.push_back(subnode);
        }
    }
    return node;
}

template <class T>
inline bool has_child(YAML::Node const &node, T const &key)
{
#ifdef YAMLCPP_NEWAPI
    return node[key].IsDefined();
#else // ifdef YAMLCPP_NEWAPI
    return !!node.FindValue(key);
#endif // ifndef YAMLCPP_NEWAPI
}

} // namespace util
} // namespace box2d_kenv

namespace YAML {

#ifdef YAMLCPP_NEWAPI

template <typename _Scalar, int _Dim, int _Mode, int _Options>
struct convert<Eigen::Matrix<_Scalar, _Dim, _Mode, _Options> > {
    typedef Eigen::Matrix<_Scalar, _Dim, _Mode, _Options> MatrixType;

    static Node encode(MatrixType const &matrix)
    {
        YAML::Node node(NodeType::Sequence);

        if (MatrixType::IsVectorAtCompileTime) {
            node.SetTag("Vector");

            for (int i = 0; i < matrix.size(); ++i) {
                node.push_back(Node(matrix[i]));
            }
        } else {
            node.SetTag("Matrix");

            for (int r = 0; r < matrix.rows(); ++r) {
                Node row(NodeType::Sequence);

                for (int c = 0; c < matrix.cols(); ++c) {
                    row.push_back(matrix(r, c));
                }

                node.push_back(row);
            }
        }

        return node;
    }

    static bool decode(
        YAML::Node const &node,
        Eigen::Matrix<_Scalar, _Dim, _Mode, _Options> &matrix)
    {
        if (node.Tag() == "Vector" || node.Tag() == "Matrix") {
            box2d_kenv::util::deserialize(node, matrix);
            return true;
        } else {
            return false;
        }
    }
};

template <typename _Scalar, int _Dim, int _Mode, int _Options>
struct convert<Eigen::Transform<_Scalar, _Dim, _Mode, _Options> > {
    typedef Eigen::Transform<_Scalar, _Dim, _Mode, _Options> TransformType;
    typedef typename TransformType::MatrixType MatrixType;

    static Node encode(TransformType const &transform)
    {
        return convert<MatrixType>::encode(transform.matrix());
    }

    static bool decode(Node const &node, TransformType &transform)
    {
        // TODO: Should I change the tag to !Transform?
        return convert<MatrixType>::decode(node, transform.matrix());
    }
};

template <class T>
inline void operator>>(Node const &node, T &value)
{
    value = node.as<T>();
}

#else // ifdef YAMLCPP_NEWAPI

template <class Derived>
inline YAML::Emitter &operator<<(YAML::Emitter &emitter, Eigen::MatrixBase<Derived> const &matrix)
{
    if (Eigen::MatrixBase<Derived>::IsVectorAtCompileTime) {
        emitter << YAML::LocalTag("Vector")
                << YAML::Flow
                << YAML::BeginSeq;

        for (int i = 0; i < matrix.size(); ++i) {
            emitter << matrix(i, 0);
        }

        emitter << YAML::EndSeq;
    } else {
        emitter << YAML::LocalTag("Matrix")
                << YAML::BeginSeq;

        for (int r = 0; r < matrix.rows(); ++r) {
            emitter << YAML::Flow
                    << YAML::BeginSeq;
            for (int c = 0; c < matrix.cols(); ++c) {
                emitter << matrix(r, c);
            }

            emitter << YAML::EndSeq;
        }

        emitter << YAML::EndSeq;
    }
    return emitter;
}

inline YAML::Emitter &operator<<(YAML::Emitter &emitter, Eigen::Affine2d const &pose)
{
    return emitter << pose.matrix();
}

inline YAML::Emitter &operator<<(YAML::Emitter &emitter, Eigen::Affine3d const &pose)
{
    return emitter << pose.matrix();
}

inline void operator>>(YAML::Node const &node, Eigen::Affine2d &pose)
{
    deserialize(node, pose.matrix());
}

inline void operator>>(YAML::Node const &node, Eigen::Affine3d &pose)
{
    deserialize(node, pose.matrix());
}

#endif // ifndef YAMLCPP_NEWAPI

} // namespace YAML

#endif // ifndef YAML_UTILS_H_
