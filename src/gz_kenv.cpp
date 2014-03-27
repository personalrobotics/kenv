#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/assert.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/typeof/typeof.hpp>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/parser.h>
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "sdf/sdf.hh"
#include <iostream>
#include "gz_kenv.h"
#include "gazebo_conversions.h"

using namespace gazebo;

namespace kenv {
  
  //Gazebo Link
  /////////////////////////////////////////////////////////////////
  
  GazeboLink::GazeboLink(boost::weak_ptr<GazeboObject> object, physics::LinkPtr link)
    : object_(object)
    , link_(link)
  {
    BOOST_ASSERT(link);
  }
  Object::Ptr GazeboLink::getObject(void) const
  {
    return object_.lock();
  }
  
  std::string GazeboLink::getName(void) const
  {
    return link_->GetName();
  }
  
  Eigen::Affine3d GazeboLink::getTransform(void) const
  {
    math::Pose pose = link_->GetWorldCoGPose();
    return toEigen(pose);
  }
  
  void GazeboLink::enable(bool flag)
  {
    link_->SetEnabled(flag);
  }
  
  AlignedBox3d GazeboLink::computeLocalAABB() {
	 return AlignedBox3d();
  }
  
  //Gazebo Model
  /////////////////////////////////////////////////////////////////////////
  
  GazeboObject::GazeboObject(boost::weak_ptr<GazeboEnvironment> parent, physics::ModelPtr kinbody,
                             std::string const &type)
      : parent_(parent)
      , model_(kinbody)
      , type_(type)
  {
    BOOST_ASSERT(!parent_.expired());
    BOOST_ASSERT(model_);
  }
  void GazeboObject::initialize(void) 
  {
    GazeboObject::Ptr this_object = shared_from_this();
    physics::Link_V all_links = model_->GetLinks();
    BOOST_FOREACH (physics::LinkPtr gz_link, all_links)
    {
      std::string const name = gz_link->GetName();
      GazeboLink::Ptr link = boost::make_shared<GazeboLink>(this_object, gz_link);
      links_.insert(std::make_pair(name,link));
    }
  }

  Environment::Ptr GazeboObject::getEnvironment(void) const {
    return parent_.lock();
  }     

  std::string GazeboObject::getName(void) const
  {
    return model_->GetName();
  }

  std::string GazeboObject::getType(void) const
  {
    return type_;
  }

  std::string GazeboObject::getKinematicsGeometryHash(void) const
  {
    return "";//Doesnt appear to be useful
  }

  physics::ModelPtr GazeboObject::getModelPtr(void) const
  {
    return model_;
  }

  bool GazeboObject::checkCollision(Object::ConstPtr entity, std::vector<Contact> *contacts, 
            std::vector<std::pair<Link::Ptr, Link::Ptr> > *links) const
  {
    GazeboObject::ConstPtr gz_entity = boost::dynamic_pointer_cast<GazeboObject const>(entity);
    BOOST_ASSERT(gz_entity);

    physics::WorldPtr world = model_->GetWorld();
    physics::ModelPtr other_model = gz_entity->model_;
    physics::PhysicsEnginePtr engine = world->GetPhysicsEngine();
    unsigned int num_contacts = engine->GetContactManager()->GetContactCount();
    if(num_contacts > 0)
    {
      for(int i = 0; i < num_contacts; i++)
      {
        physics::Collision* c1 = engine->
          GetContactManager()->GetContact(i)->collision1;
        physics::Collision* c2 = engine->
          GetContactManager()->GetContact(i)->collision2;

        if(((c1->GetModel()->GetName() == model_->GetName()) &&
            (c2->GetModel()->GetName() == other_model->GetName())) ||
           ((c2->GetModel()->GetName() == model_->GetName()) &&
            (c1->GetModel()->GetName() == other_model->GetName())))
        {
          if(links)
          {
            Link::Ptr link1 = getLink(c1->GetLink()->GetName());
            Link::Ptr link2 = getLink(c2->GetLink()->GetName());
            links->push_back(std::make_pair(link1, link2));
          }
          if(contacts)
          {
            Contact contact;
            int count = engine->GetContactManager()->GetContact(i)->count;
            math::Vector3 n[32] 
              = engine->GetContactManager()->GetContact(i)->normals;
            math::Vector3 pos[32] 
              = engine->GetContactManager()->GetContact(i)->positions;    
            for(int j = 0; j < count; j++)
            {
              contact.normal = -1*toEigen(n[j]);
              contact.position = toEigen(pos[j]);
              contacts->push_back(contact);  
            }      
            
          }
          return true;
        }
      }
    }
    return false;

  } 

  void GazeboObject::enable(bool flag)
  {
    model_->SetEnabled(flag);
  }

  void GazeboObject::setVisible(bool flag)
  {
    return; 
  }

  Eigen::Affine3d GazeboObject::getTransform(void) const
  {
    math::Pose p = model_->GetRelativePose();
    math::Vector3 v = p.pos;
    math::Quaternion r = p.rot;
    return toEigen(p);
  }

  void GazeboObject::setTransform(Eigen::Affine3d const &tf)
  {
    math::Pose p = toGZ(tf);
    model_->SetRelativePose(p);
  }

  AlignedBox3d GazeboObject::getAABB(void) const
  {
    AlignedBox3d b; //Dummy function
    return b;
  }

  void GazeboObject::setTransparency(double x)
  {
    return;
  }

  void GazeboObject::setColor(Eigen::Vector4d const &color)
  {
    return;
  }

  std::vector<Link::Ptr> GazeboObject::getLinks(void) const
  {
    std::vector<Link::Ptr> links;
    links.reserve(links_.size());
    GazeboLink::Ptr link;
    std::string name;
    BOOST_FOREACH(boost::tie(name, link), links_) {
      links.push_back(link);
    }
    return links;

  }

  Link::Ptr GazeboObject::getLink(std::string const name) const
  {
    return links_.at(name);
  }

  Eigen::VectorXd GazeboObject::getDOFValues(void) const
  {
    return Eigen::VectorXd();
  }

  void GazeboObject::setDOFValues(Eigen::VectorXd const &dof_values)
  {
    return;
  }


  GazeboEnvironment::GazeboEnvironment(void)
  {}
  
  GazeboEnvironment::GazeboEnvironment(physics::WorldPtr gazebo_env)
  {
    env_ = gazebo_env;
  }
  

  Object::Ptr GazeboEnvironment::getObject(std::string const &name)
  {
      physics::ModelPtr kinbody = env_->GetModel(name);
      if (kinbody) {
          BOOST_AUTO(result, objects_.insert(std::make_pair(kinbody, GazeboObject::Ptr())));
          if (result.second) {
              // TODO: Use a special name for "unknown" types.
              result.first->second = boost::make_shared<GazeboObject>(shared_from_this(), kinbody, "");
          }
          return result.first->second;
      }
      return Object::Ptr();
  }
  
  Object::Ptr GazeboEnvironment::createObject(std::string const &type, std::string const &name, bool anonymous)
  {
    std::string loading = types_.at(type);
    return Object::Ptr();

  }

  void GazeboEnvironment::remove(Object::Ptr object)
  {
    return;
  }

  boost::shared_ptr<void> GazeboEnvironment::drawLine(Eigen::Vector3d const &start, Eigen::Vector3d const &end,
                                                  double width, Eigen::Vector4d const &color)
  {
  }

  boost::shared_ptr<void> GazeboEnvironment::drawLineStrip(std::vector<Eigen::Vector3d> const &points,
                                                       double width, Eigen::Vector4d const &color)
  {
  }

  boost::shared_ptr<void> GazeboEnvironment::drawLineList(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > const &lines,
                                                      double width, Eigen::Vector4d const &color)
  {
  }


  boost::shared_ptr<void> GazeboEnvironment::drawArrow(Eigen::Vector3d const &start, Eigen::Vector3d const &end,
                                                   double width, Eigen::Vector4d const &color)
  {
    
  }

  boost::shared_ptr<void> GazeboEnvironment::drawPoints(std::vector<Eigen::Vector3d> const &points,
                                                    float point_size, Eigen::Vector4d const &color)
  {
  }

  boost::shared_ptr<void> GazeboEnvironment::drawPlane(Eigen::Affine3d const &origin,
                                                   float width, float height,
                                                   boost::multi_array<float, 3> const &texture)
  {
  }

  physics::WorldPtr GazeboEnvironment::getGazeboEnvironment(void)
  {
    return env_;
  }

  void GazeboEnvironment::addType(std::string const &type, std::string const &path)
  {
    BOOST_AUTO(it, types_.insert(std::make_pair(type, path)));
    
  }
}