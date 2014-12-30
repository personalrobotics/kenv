#ifndef BOX2DLINK_H_
#define BOX2DLINK_H_
#include <vector>
#include <set>
#include <string>
#include <Eigen/Dense>
#include "box2d_kenv.hh"

class b2Body;
class b2Fixture;

namespace box2d_kenv {

class Box2DLink {
public:
    Box2DLink(Box2DBodyPtr body, std::string const &name, b2Body *b2_body);

    Box2DWorldPtr world() const;
    Box2DBodyPtr parent_body() const;
    Box2DLinkPtr parent_link() const;
    Box2DJointPtr parent_joint() const;
    std::vector<Box2DJointPtr> child_joints() const;

    b2Body *b2_body();
    b2Body const *b2_body() const;

    Eigen::Affine2d pose() const;
    void set_pose(Eigen::Affine2d const &pose);

    Eigen::Vector3d twist() const;
    void set_twist(Eigen::Vector3d const &twist) const;

    std::vector<b2Fixture const *> fixtures() const;

    void AddChildJoint(Box2DJointPtr const &joint);

private:
    Box2DBodyWeakPtr parent_body_;
    Box2DJointWeakPtr parent_joint_;
    std::set<Box2DJointPtr> child_joints_;

    std::string name_;
    b2Body *b2_body_;
};

}

#endif

