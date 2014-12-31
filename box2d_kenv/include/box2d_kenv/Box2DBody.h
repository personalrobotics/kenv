#ifndef BOX2DBODY_H_
#define BOX2DBODY_H_
#include <Eigen/Dense>
#include "box2d_kenv.hh"

namespace box2d_kenv {

class Box2DBody {
public:
    Box2DBody(Box2DWorldPtr const &world, std::string const &name);
    Box2DWorldPtr world() const;

    Box2DLinkPtr root_link();

    Eigen::Affine2d pose() const;
    void set_pose(Eigen::Affine2d const &pose);

    Eigen::Vector3d twist() const;
    void set_twist(Eigen::Vector3d const &twist);

private:
    std::string name_;
    Box2DWorldWeakPtr world_;
    Box2DLinkPtr root_link_;

    void CheckInitialized() const;

    friend class Box2DFactory;
};

}

#endif
