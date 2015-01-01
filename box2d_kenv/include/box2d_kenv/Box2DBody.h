#ifndef BOX2DBODY_H_
#define BOX2DBODY_H_
#include <vector>
#include <boost/unordered_map.hpp>
#include <Eigen/Dense>
#include "box2d_kenv.hh"

namespace box2d_kenv {

class Box2DBody {
public:
    Box2DBody(Box2DWorldPtr const &world, std::string const &name);
    Box2DWorldPtr world() const;

    Box2DLinkPtr root_link();

    std::string name() const;

    Eigen::Affine2d pose() const;
    void set_pose(Eigen::Affine2d const &pose);

    Eigen::Vector3d twist() const;
    void set_twist(Eigen::Vector3d const &twist);

    std::vector<Box2DLinkPtr> links() const;
    std::vector<Box2DJointPtr> joints() const;

    void Initialize(Box2DLinkPtr const &root_link);

private:
    std::string name_;
    Box2DWorldWeakPtr world_;
    Box2DLinkPtr root_link_;
    std::vector<Box2DLinkPtr> links_;
    std::vector<Box2DJointPtr> joints_;

    void CheckInitialized() const;

    void GetChildren(Box2DLinkPtr const &root_link,
                     std::vector<Box2DLinkPtr> *links,
                     std::vector<Box2DJointPtr> *joints) const;
    void SetZero(Box2DLinkPtr const &link, Eigen::Affine2d const &pose);
};

}

#endif
