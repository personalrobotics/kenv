#ifndef YAML_UTILS_H_
#define YAML_UTILS_H_
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

namespace box2d_kenv {
namespace util {

template <class Derived>
inline void operator>>(YAML::Node const &node, Eigen::MatrixBase<Derived> &matrix)
{
    if (node.Type() != YAML::NodeType::Sequence) {
        throw std::runtime_error("Matrix or vector must be a sequence.");
    } else if (node.size() < 1) {
        throw std::runtime_error(boost::str(
            boost::format("Matrix or vector must have one or more rows; has %d.")
                % node.size()));
    }

    size_t const rows = node.size();
    if (node.Tag() == "!Vector") {
        matrix.resize(rows, 1);

        for (size_t i = 0; i < rows; ++i) {
            node[i] >> matrix(i, 0);
        }
    } else if (node.Tag() == "!Matrix") {
        size_t const cols = node[0].size();
        if (cols < 1) {
            throw std::runtime_error(boost::str(
                boost::format("Matrix must have one or more columns; has %d.") % cols));
        }

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
                node[r][c] >> matrix(r, c);
            }
        }
    } else {
        throw std::runtime_error(boost::str(
            boost::format("Unknown type of matrix '%s'.") % node.Tag()));
    }
}

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

template <class Scalar, int Dim, int Mode, int Options>
inline YAML::Emitter &operator<<(YAML::Emitter &emitter,
                                 Eigen::Transform<Scalar, Dim, Mode, Options> const &pose)
{
    return emitter << pose.matrix();
}

template <class Scalar, int Dim, int Mode, int Options>
inline void operator>>(YAML::Node const &node,
                        Eigen::Transform<Scalar, Dim, Mode, Options> &pose)
{
    node >> pose.matrix();
}

}
}

#endif
