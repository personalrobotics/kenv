#ifndef BOX2DJOINT_H_
#define BOX2DJOINT_H_
#include <Eigen/Dense>
#include <Box2D/Dynamics/Joints/b2Joint.h>
#include <Box2D/Dynamics/Joints/b2RevoluteJoint.h>
#include "box2d_kenv.hh"

namespace box2d_kenv {

class Box2DJoint {
public:
    Box2DJoint(std::string const &name,
               Box2DLinkPtr parent_link,
               Box2DLinkPtr child_link,
               b2RevoluteJoint *b2_joint,
               double direction = 1.);

    std::string name() const;

    Box2DLinkPtr parent_link() const;
    Box2DLinkPtr child_link() const;

    Eigen::Affine2d origin() const;

    b2RevoluteJoint *b2_joint();
    b2RevoluteJoint const *b2_joint() const;

    double value() const;
    double velocity() const;

    void set_desired_velocity(double velocity);

private:
    std::string name_;
    Box2DLinkWeakPtr parent_link_;
    Box2DLinkPtr child_link_;
    b2RevoluteJoint *b2_joint_;
    double direction_;
};

}

#endif
