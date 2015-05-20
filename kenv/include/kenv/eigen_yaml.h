#ifndef EIGEN_YAML_H_
#define EIGEN_YAML_H_

#include <boost/format.hpp>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

template <class Derived>
inline void deserialize(YAML::Node const &node, Eigen::MatrixBase<Derived> &matrix)
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
            matrix(i, 0) = node[i].as<typename Derived::Scalar>();
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
                matrix(r, c) = node[r][c].as<typename Derived::Scalar>();
            }
        }
    } else {
        throw std::runtime_error(boost::str(
            boost::format("Unknown type of matrix '%s'.") % node.Tag()));
    }
}


// TODO: I should really specialize the YAML::convert class.
#ifdef YAMLCPP_NEWAPI

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

namespace YAML {

template<>
struct convert<Eigen::Affine2d> {
    static Node encode(Eigen::Affine2d const &pose)
    {
        return serialize(pose.matrix());
    }

    static bool decode(YAML::Node const &node, Eigen::Affine2d &pose)
    {
        deserialize(node, pose.matrix());
        return true;
    }
};

template<>
struct convert<Eigen::Affine3d> {
    static Node encode(Eigen::Affine3d const &pose)
    {
        return serialize(pose.matrix());
    }

    static bool decode(YAML::Node const &node, Eigen::Affine3d &pose)
    {
        deserialize(node, pose.matrix());
        return true;
    }
};

}

#else

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
#endif

#endif
