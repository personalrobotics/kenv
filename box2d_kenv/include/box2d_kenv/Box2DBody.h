#ifndef BOX2DBODY_H_
#define BOX2DBODY_H_
#include <vector>
#include <boost/enable_shared_from_this.hpp>
#include <boost/unordered_map.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "box2d_kenv.hh"
#include "Box2DBody.h"
#include "Box2DFactory.h"

namespace box2d_kenv {

class Box2DBody : public boost::enable_shared_from_this<Box2DBody> {
public:
    Box2DBody(Box2DWorldPtr const &world, std::string const &name);
    virtual ~Box2DBody();

    Box2DWorldPtr world() const;

    Box2DLinkPtr root_link();

    std::string name() const;

    Eigen::Affine2d pose() const;
    void set_pose(Eigen::Affine2d const &pose);

    Eigen::Vector3d twist() const;
    void set_twist(Eigen::Vector3d const &twist);

    Box2DLinkPtr GetLink(std::string const &name);
    Box2DLinkConstPtr GetLink(std::string const &name) const;

    Box2DJointPtr GetJoint(std::string const &name);
    Box2DJointConstPtr GetJoint(std::string const &name) const;

    std::vector<Box2DLinkPtr> links();
    std::vector<Box2DLinkConstPtr> links() const;

    std::vector<Box2DJointPtr> joints();
    std::vector<Box2DJointConstPtr> joints() const;

    std::vector<Box2DSensorPtr> sensors();
    std::vector<Box2DSensorConstPtr> sensors() const;

    void Initialize(Box2DLinkPtr const &root_link);
    void CreateSensors(std::istream &stream);
    void CreateSensors(std::string const &path);

private:
    std::string name_;
    Box2DWorldWeakPtr world_;
    Box2DLinkPtr root_link_;
    std::vector<Box2DLinkPtr> links_;
    std::vector<Box2DJointPtr> joints_;
    std::vector<Box2DSensorPtr> sensors_;
    boost::unordered_map<std::string, Box2DLinkPtr> links_map_;
    boost::unordered_map<std::string, Box2DJointPtr> joints_map_;
    // TODO: Allow for named sensors.

    void CheckInitialized() const;

    void GetChildren(Box2DLinkPtr const &root_link);
    void SetZero(Box2DLinkPtr const &link, Eigen::Affine2d const &pose);
};

}

#endif
