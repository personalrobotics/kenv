#ifndef OR_CONVERSIONS_H_
#define OR_CONVERSIONS_H_

#include <boost/assign/std/vector.hpp>
#include <Eigen/Dense>
#include <openrave/openrave.h>

namespace kenv {

inline Eigen::Vector3d toEigen3(OpenRAVE::Vector const &or_v)
{
    Eigen::Vector3d eigen_v;
    eigen_v << or_v.x, or_v.y, or_v.z;
    return eigen_v;
}

inline Eigen::Affine3d toEigen(OpenRAVE::Transform const &or_tf)
{
    OpenRAVE::TransformMatrix or_matrix(or_tf);
    Eigen::Affine3d eigen_tf = Eigen::Affine3d::Identity();
    eigen_tf.linear() << or_matrix.m[0], or_matrix.m[1], or_matrix.m[2],
                         or_matrix.m[4], or_matrix.m[5], or_matrix.m[6],
                         or_matrix.m[8], or_matrix.m[9], or_matrix.m[10];
    eigen_tf.translation() << or_matrix.trans.x, or_matrix.trans.y, or_matrix.trans.z;
    return eigen_tf;
}

inline OpenRAVE::Vector toOR(Eigen::Vector3d const &eigen_v)
{
    return OpenRAVE::Vector(eigen_v[0], eigen_v[1], eigen_v[2]);
}

inline OpenRAVE::Vector toOR(Eigen::Vector4d const &eigen_v)
{
    return OpenRAVE::Vector(eigen_v[0], eigen_v[1], eigen_v[2], eigen_v[3]);
}

inline OpenRAVE::GraphHandlePtr plot3(OpenRAVE::EnvironmentBasePtr env,
                               std::vector<Eigen::Vector3d> const &points,
                               float point_size, Eigen::Vector4d const &color)
{
    std::vector<float> raw_points(3 * points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        raw_points[3 * i + 0] = points[i][0];
        raw_points[3 * i + 1] = points[i][1];
        raw_points[3 * i + 2] = points[i][2];
    }

    int const num_points = static_cast<int>(points.size());
    return env->plot3(&raw_points.front(), num_points, 3 * sizeof(float), point_size); //, toOR(color));
}

inline OpenRAVE::GraphHandlePtr drawArrow(OpenRAVE::EnvironmentBasePtr env,
                                   Eigen::Vector3d const &start, Eigen::Vector3d const &end,
                                   float width, Eigen::Vector4d const &color)
{
    return env->drawarrow(toOR(start), toOR(end), width, toOR(color));
}

}

#endif
