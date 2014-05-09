#ifndef GZ_KENV_H_
#define GZ_KENV_H_

#include <boost/enable_shared_from_this.hpp>
#include <boost/filesystem.hpp>
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "sdf/sdf.hh"
#include "kenv/Environment.h"

using namespace gazebo;

namespace kenv {
  class GazeboEnvironment;
  class GazeboObject;

class GazeboLink : virtual public Link {
public:
  typedef boost::shared_ptr<Link> Ptr;
  typedef boost::shared_ptr<Link const> ConstPtr;
  
  
  GazeboLink(boost::weak_ptr<GazeboObject> robot, physics::LinkPtr link);
  virtual Object::Ptr getObject(void) const;

  
  virtual std::string getName(void) const;
  virtual Eigen::Affine3d getTransform(void) const;
  virtual void enable(bool flag);

  AlignedBox3d computeLocalAABB();
  
  private:
    boost::weak_ptr<GazeboObject> object_;
    physics::LinkPtr link_;

};

class GazeboObject : virtual public Object, public boost::enable_shared_from_this<GazeboObject>{
public:
  typedef boost::shared_ptr<GazeboObject> Ptr;
  typedef boost::shared_ptr<GazeboObject const> ConstPtr;
  
  GazeboObject(boost::weak_ptr<GazeboEnvironment> ptr, physics::ModelPtr, std::string const &type);
  virtual void initialize(void);
  physics::ModelPtr getModelPtr(void) const;
  
  virtual Environment::Ptr getEnvironment(void) const;
  virtual std::string getName(void) const;
  virtual std::string getType(void) const;
  virtual std::string getKinematicsGeometryHash(void) const;
  
  virtual bool checkCollision(Object::ConstPtr entity, std::vector<Contact> *contacts = NULL, 
            std::vector<std::pair<Link::Ptr, Link::Ptr> > *links = NULL) const;
  
  virtual std::vector<Link::Ptr> getLinks(void) const;
  virtual Link::Ptr getLink(std::string const name) const;
  
  virtual void enable(bool flag);
  virtual void setVisible(bool flag);

  virtual Eigen::Affine3d getTransform(void) const;
  virtual void setTransform(Eigen::Affine3d const &tf);
  virtual AlignedBox3d getAABB(void) const;

  virtual Eigen::VectorXd getDOFValues(void) const;
  virtual void setDOFValues(Eigen::VectorXd const &dof_values);

  virtual void setColor(Eigen::Vector4d const &color);
  virtual void setTransparency(double p);
private:
  boost::weak_ptr<GazeboEnvironment> parent_;
  physics::ModelPtr model_;
  std::map<std::string, GazeboLink::Ptr> links_;
  std::string type_;  
};


class GazeboEnvironment : public Environment, public boost::enable_shared_from_this<GazeboEnvironment> {
public:
    typedef boost::shared_ptr<GazeboEnvironment> Ptr;
    typedef boost::shared_ptr<GazeboEnvironment const> ConstPtr;
    
    GazeboEnvironment();
    explicit GazeboEnvironment(std::string const &path);
    explicit GazeboEnvironment(physics::WorldPtr world);
    physics::WorldPtr getGazeboEnvironment(void);
    
    virtual Object::Ptr getObject(std::string const &name);

    virtual Object::Ptr createObject(std::string const &type, std::string const &name, bool anonymous = false);
    virtual void remove(Object::Ptr object);
    virtual void runWorld(int steps);

    virtual Handle drawLine(Eigen::Vector3d const &start, Eigen::Vector3d const &end,
                            double width, Eigen::Vector4d const &color);
    virtual Handle drawLineList(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > const &lines,
                                double width, Eigen::Vector4d const &color);
    virtual Handle drawLineStrip(std::vector<Eigen::Vector3d> const &points,
                                 double width, Eigen::Vector4d const &color);
    virtual Handle drawArrow(Eigen::Vector3d const &start, Eigen::Vector3d const &end,
                             double width, Eigen::Vector4d const &color);
    virtual Handle drawPoints(std::vector<Eigen::Vector3d> const &points,
                              float point_size, Eigen::Vector4d const &color);

    virtual Handle drawPlane( const Eigen::Affine3d& origin, float width, float height,
    						const boost::multi_array<float,3>& texture);
    
    void addType(std::string const &type, std::string const &kinbody_path);
private:
    physics::WorldPtr env_;
    std::map<std::string, std::string> types_;
    std::map<physics::ModelPtr, GazeboObject::Ptr> objects_;
    bool addObject(GazeboObject::Ptr object, std::string const name, bool anonymous);
};


};

#endif
