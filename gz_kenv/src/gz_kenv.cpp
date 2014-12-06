#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/assert.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/typeof/typeof.hpp>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/parser.h>
#include <gazebo/gazebo.hh>
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
    throw std::runtime_error("compute AABB not implemented");
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
    throw std::runtime_error("getKinematicGeometryHash not implemented");
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
    throw std::runtime_error("aabb not implemented");
  }

  void GazeboObject::setTransparency(double x)
  {
    throw std::runtime_error("transperancy not implemented");
  }

  void GazeboObject::setColor(Eigen::Vector4d const &color)
  {
    throw std::runtime_error("setColor not implemented");
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
    Eigen::VectorXd dof_values(model_->GetJoints().size());
    for (size_t i = 0; i < model_->GetJoints().size(); ++i) {
        dof_values[i] = *(model_->GetJoints().at(i)->GetAngle(0));
    }
    return dof_values;
  }

  void GazeboObject::setDOFValues(Eigen::VectorXd const &dof_values)
  {
    std::cout << "Set DOF " << model_->GetName() << " " << (dof_values.size()) << " " << model_->GetLinks().size() << '\n'; 
    // BOOST_ASSERT(dof_values.size() == static_cast<int>(model_->GetJoints().size()));

    bool invalidated = false;

    for (size_t i = 0; i < model_->GetJoints().size(); ++i) {
        if (*(model_->GetJoints().at(i)->GetAngle(0)) != dof_values[i]) {
            // FIXME: This triggers |joints_| updates. It only needs to trigger one.
            model_->GetJoints().at(i)->SetAngle(0,math::Angle(dof_values[i]));
            invalidated = true;
        }
    }

    // if (invalidated) {
    //     cached_geometry_.reset();
    // }
  }

  GazeboEnvironment::GazeboEnvironment(void)
  {
    static std::string const empty_world_xml = "<?xml version=\"1.0\"?><sdf version=\"1.4\"><world name=\"autogenerated\"></world></sdf>";

    // This is black magic that we scraped together from the Gazebo source
    // code. I have no idea what any of these functions do, but creating the
    // world SEGFAULTs if you remove any of them. Also, note that initFile()
    // and readFile() do two very different things (although I can't say what
    // either of them does).
    sdf::SDFPtr sdf(new sdf::SDF);
    if (!sdf::init(sdf)) {
      throw std::runtime_error("Failed initializing SDF.");
    }
    if (!sdf::readString(empty_world_xml, sdf)) {
      throw std::runtime_error("Failed loading empty world XML.");
    }

    env_ = gazebo::physics::create_world();
    if (!env_) {
      throw std::runtime_error("Creating world failed.");
    }
    
    gazebo::physics::load_world(env_, sdf->root->GetElement("world"));
    gazebo::physics::init_world(env_);

    // We need to run one simulation step to initialize the world.
    runWorld(1);
  }
  
  GazeboEnvironment::GazeboEnvironment(physics::WorldPtr gazebo_env)
  {
    env_ = gazebo_env;
  }

  GazeboEnvironment::GazeboEnvironment(std::string const &path)
  {
    env_ = gazebo::loadWorld(path);
    if (!env_) {
      throw std::runtime_error(boost::str(boost::format("Loading world [%s] failed.") % path));
    }
    std::cout << "Successfully created world." << std::endl;
    runWorld(1);
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
    // InsertModelFile returns void and doesn't throw an exception if loading
    // the model fails. We need to track the models before and after the
    // function call to determine which was created.
    size_t const num_models_before = env_->GetModelCount();
    physics::Model_V models_before = env_->GetModels();

    // FIXME: If this fails, Gazebo stalls for a long time and returns a
    // MemoryError.
    env_->InsertModelFile(type);

    // We need to run one simulation step to update the world. Otherwise, the
    // list of models will not be updated.
    runWorld(1);

    size_t const num_models_after = env_->GetModelCount();
    physics::Model_V models_after = env_->GetModels();

    if (num_models_after == num_models_before) {
      throw std::runtime_error(boost::str(
        boost::format("Failed loading model '%s'.") % type));
    } else if (num_models_after != num_models_before + 1) {
      throw std::runtime_error(boost::str(
        boost::format("Expected %d models; found %d. Something is seriously wrong.")
          % (num_models_before + 1) % num_models_after
        )
      );
    }

    std::set<physics::ModelPtr> models_before_set(
      models_before.begin(), models_before.begin() + num_models_before);
    std::set<physics::ModelPtr> models_after_set(
      models_after.begin(), models_after.begin() + num_models_after);
    std::vector<physics::ModelPtr> new_models;

    std::set_difference(
      models_after_set.begin(), models_after_set.end(),
      models_before_set.begin(), models_before_set.end(),
      std::back_inserter(new_models)
    );

    BOOST_ASSERT(new_models.size() == 1);
    physics::ModelPtr const &model = new_models[0];
    model->SetName(name);

    // Update the name.
    runWorld(1);

    return getObject(name);
  }

  void GazeboEnvironment::remove(Object::Ptr object)
  {
    return;
  }
  void GazeboEnvironment::runWorld(int steps)
  {
    gazebo::runWorld(env_,steps);
   }

  boost::shared_ptr<void> GazeboEnvironment::drawLine(Eigen::Vector3d const &start, Eigen::Vector3d const &end,
                                                  double width, Eigen::Vector4d const &color)
  {
    throw std::runtime_error("not implemented");
  }

  boost::shared_ptr<void> GazeboEnvironment::drawLineStrip(std::vector<Eigen::Vector3d> const &points,
                                                       double width, Eigen::Vector4d const &color)
  {
    throw std::runtime_error("not implemented");
  }

  boost::shared_ptr<void> GazeboEnvironment::drawLineList(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > const &lines,
                                                      double width, Eigen::Vector4d const &color)
  {
    throw std::runtime_error("not implemented");
  }

  boost::shared_ptr<void> GazeboEnvironment::drawArrow(Eigen::Vector3d const &start, Eigen::Vector3d const &end,
                                                   double width, Eigen::Vector4d const &color)
  {
    throw std::runtime_error("not implemented");
  }

  boost::shared_ptr<void> GazeboEnvironment::drawPoints(std::vector<Eigen::Vector3d> const &points,
                                                    float point_size, Eigen::Vector4d const &color)
  {
    throw std::runtime_error("not implemented");
  }

  boost::shared_ptr<void> GazeboEnvironment::drawPlane(Eigen::Affine3d const &origin,
                                                   float width, float height,
                                                   boost::multi_array<float, 3> const &texture)
  {
    throw std::runtime_error("not implemented");
  }

  physics::WorldPtr GazeboEnvironment::getGazeboEnvironment(void)
  {
    return env_;
  }

  void GazeboEnvironment::addType(std::string const &type, std::string const &path)
  {
    BOOST_AUTO(it, types_.insert(std::make_pair(type, path)));
    
  }

  void GazeboEnvironment::setupServer()
  {
	  gazebo::setupServer();
  }
}
