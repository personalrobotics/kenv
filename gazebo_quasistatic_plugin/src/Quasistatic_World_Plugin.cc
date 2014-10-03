#include "Quasistatic_World_Plugin.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "Quasistatic_World_Plugin.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/transport/transport.hh" 
#include "sdf/sdf.hh"
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/assert.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/typeof/typeof.hpp>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include "ros/ros.h"
#include "kenv/Environment.h"
#include "kenv/CollisionChecker.h"

#include "gz_kenv/gz_kenv.h"
#include "quasistatic_pushing/QuasistaticPushingModel.h"
#include "gazebo/transport/transport.hh"
#include "set_pusher_request.pb.h"
#include "add_pushee_request.pb.h"
using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(Quasistatic_World_Plugin)

Quasistatic_World_Plugin::Quasistatic_World_Plugin()
{
  int argc = 0;
  ros::init(argc, NULL, "Quasistatic_World_Plugin");

  auto collision_checker = boost::make_shared<kenv::DefaultCollisionChecker>();
  simulator_ = boost::make_shared<quasistatic_pushing::QuasistaticPushingModel>(
    collision_checker, .0001,.0001);
}

void Quasistatic_World_Plugin::Load(physics::WorldPtr parent, sdf::ElementPtr sdf){

  //Set up Transport Layer
  node = transport::NodePtr(new transport::Node());
  node->Init(parent->GetName());
  pusherAssignmentSubscriber = node -> Subscribe("quasistatic_set_pusher", &Quasistatic_World_Plugin::setPusher, this);
  pusheeAssignmentSubscriber = node -> Subscribe("quasistatic_add_pushee", &Quasistatic_World_Plugin::addPushee, this);
  double mu = 0.5;
  double c = 0.8;

  world = parent;
  kenv_world = boost::make_shared<kenv::GazeboEnvironment>(world);
  physics::Model_V models= world -> GetModels();


  
  sdf::ElementPtr child_sdf = sdf->GetFirstElement();
  while (child_sdf) {
    std::string const child_name = child_sdf->GetName();
    // Load a single pusher from SDF.
    if (child_name == "pusher") {
      std::string pusher_name;
      if (pusher_) {
        throw std::runtime_error("Duplicate <pusher>.");
      } else if (!child_sdf->GetValue() || !child_sdf->GetValue()->Get(pusher_name)) {
        throw std::runtime_error("<pusher> must contain a model name.");
      }

      physics::ModelPtr pusher = parent->GetModel(pusher_name);
      if (!pusher) {
        throw std::runtime_error(boost::str(
          boost::format("Invalid pusher: there is no model named '%s' in the world.")
            % pusher_name));
      }

      set_pusher(pusher);
      std::cout << "SET pusher " << pusher_name << std::endl;
    }
    // Load [potentially several] pushees from SDF.
    else if (child_name == "pushee") {
      double mu = 0.5, c = 0.8;
      std::string pushee_name;

      sdf::ParamPtr name_param = child_sdf->GetValue();
      sdf::ParamPtr mu_param = child_sdf->GetAttribute("mu");
      sdf::ParamPtr c_param = child_sdf->GetAttribute("c");

      if (!name_param || !name_param->Get(pushee_name)) {
        throw std::runtime_error("<pushee> must contain a model name.");
      }
      // TODO: Why don't attributes work properly?
#if 0
      else if (!mu_param || !mu_param->Get(mu)) {
        throw std::runtime_error(boost::str(
          boost::format("Pushee '%s' must have attribute 'mu'.") % pushee_name));
      } else if (mu < 0.) {
        throw std::runtime_error(boost::str(
          boost::format("Pushee '%s' has invalid attribute 'mu': must be non-negative; got %f.")
            % pushee_name % mu));
      } else if (c_param || !c_param->Get(c)) {
        throw std::runtime_error(boost::str(
          boost::format("Pushee '%s' must have attribute 'c'.") % pushee_name));
      } else if (c <= 0.) {
          throw std::runtime_error(boost::str(
            boost::format("Pushee '%s' has invalid attribute 'c': must be positive; got %f.")
              % pushee_name % c));
      }
#endif

      physics::ModelPtr model = parent->GetModel(pushee_name);
      if (!model) {
        throw std::runtime_error(boost::str(
          boost::format("Invalid pushee: there is no model named '%s' in the world.")
            % pushee_name));
      }

      add_pushee(model, mu, c);
      std::cout << "ADD pushee " << pushee_name << std::endl;
    } else {
      throw std::runtime_error(boost::str(boost::format("Unknown SDF tag <%s>.") % child_name));
    }

    child_sdf = child_sdf->GetNextElement();
  }
  /* Initialize with active gazebosim and no quasistatic */
  has_contact_ = false;
  objects_loaded_ = false;
  cur_pushee_set = false;

  /* Used for testing purposes */
  pusher_vel_[0] = 0.03;
  pusher_vel_[1] = 0;
  pusher_vel_[2] = 0;

  /* Set up phyiscs Engine */
  // TODO: This should be handled by the world file.
  engine = world->GetPhysicsEngine();
  engine->SetGravity(math::Vector3(0., 0., 0.));
  engine->SetRealTimeUpdateRate(1e9);
  engine->SetMaxStepSize(0.001);

  /* Set up contact manager to get contacts */
  contact_manager_ = engine->GetContactManager();

  /* Listen to 'step' events */
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&Quasistatic_World_Plugin::OnUpdate, this, _1));
  world->EnableAllModels();
}

/*
 * Sets the pusher to the named object
 * 
 */

void Quasistatic_World_Plugin::addPushee(AddPusheeRequestPtr &msg)
{
  double mu = 0.5;
  double c = 0.8;

  std::string pushee_name = msg->pushee();
  physics::ModelPtr model = world->GetModel(pushee_name);
  
  //Check if this is the pusher

  if(pusher_ && !pushee_name.compare(pusher_ -> GetName()))
     pusher_ = boost::shared_ptr<physics::Model>();
  if(model)
     add_pushee(model, mu, c); 
}

void Quasistatic_World_Plugin::setPusher(SetPusherRequestPtr &msg)
{
  double mu = 0.5;
  double c = 0.8;
  std::string pusher_name = msg->pusher();
  physics::ModelPtr model = world->GetModel(pusher_name);

  if(pusher_)
    add_pushee(pusher_,  mu, c);

  set_pusher(model);
  pushees_.erase(model); 
}

//Note that this implies that that only one object can be updated at a time
void Quasistatic_World_Plugin::OnUpdate(common::UpdateInfo const &info)
{
  if (!pusher_ || pushees_.empty()) {
    return;
  }
   // pusher_->SetLinearVel(math::Vector3(0.03,0,0));

   boost::unordered_map<physics::ModelPtr, physics::Contact> contacts = get_pusher_contacts(); 
  
  bool has_contact = contact_manager_ -> GetContactCount();
  if(has_contact && pusher_){

    //SET UP CONTROLS
     double mu = 0.5;
     double c = 0.5;
     math::Vector3 vv = pusher_->GetRelativeLinearVel();

     Eigen::Vector2d v(vv.x,vv.y);
     quasistatic_pushing::Action a(v, 0.0,.001);
     simulator_->Simulate_Step(kenv_world->getObject(pusher_->GetName()), 
                               kenv_world->getObject(cur_pushee->GetName()),  a, mu, c, false);


     math::Pose pose = pusher_->GetRelativePose();
     math::Vector3 trans = pose.pos;
     set_ode(pusher_, true);
     set_ode(cur_pushee, true);
     cur_pushee->SetAngularVel(math::Vector3(0,0,0));

     

  } else {
     math::Pose pose = pusher_->GetRelativePose();
     math::Vector3 trans = pose.pos;
    if(cur_pushee_set){
       cur_pushee->SetLinearVel(math::Vector3(0,0,0));
       cur_pushee->SetAngularVel(math::Vector3(0,0,0));
    }
    //Comment this out when integrating with state estimator

    // pusher_->SetLinearVel(math::Vector3(pusher_vel_[0],pusher_vel_[1], pusher_vel_[2]));
    pusher_->SetAngularVel(math::Vector3(0,0,0));

    //THIS IS FOR STATE ESTIMATOR
    // pusher->SetLinearVel(pusher->GetRelativeLinearVel());
    // pusher->SetAngularVel(pusher->GetRelativeAngularVel());
  }
  ros::spinOnce();
}

/* check if pusher has contact  */
boost::unordered_map<physics::ModelPtr, physics::Contact> Quasistatic_World_Plugin::get_pusher_contacts()
{
  std::vector<physics::Contact*> contacts_tmp = contact_manager_->GetContacts();
  std::vector<physics::Contact*> contacts;
  //Filter contacts 
  if(contact_manager_ -> GetContactCount() == 0){
     cur_pushee_set = false;
  }
  for(int i = 0; i < contact_manager_ -> GetContactCount(); i++){
    contacts.push_back(contacts_tmp.at(i));
  }
  boost::unordered_map<physics::ModelPtr, physics::Contact> pusher_contacts;

  for (physics::Contact const *const contact : contact_manager_->GetContacts()) {
    cur_pushee_set = false;
    physics::Collision *const collision1 = contact->collision1;
    physics::Collision *const collision2 = contact->collision2;

    // Normalize the contact to put the pusher first.
    physics::Contact normalized_contact;
    if (contact->collision1->GetModel() == pusher_) {
      normalized_contact = *contact;
    } else if (contact->collision2->GetModel() == pusher_) {
      normalized_contact = *contact;
      std::swap(normalized_contact.collision1, normalized_contact.collision2);
    } else {
      continue;
    }

    // Check if contact is with the pushee.
    physics::ModelPtr pushee = normalized_contact.collision2->GetModel();
    try {
        pushees_[pushee];
        cur_pushee = pushee;
        cur_pushee_set = true;
        break;
    } catch (int e)
    {
      continue; 
    }
    

    // TODO: Do something with the contacts.
    auto result = pusher_contacts.insert(std::pair<physics::ModelPtr, physics::Contact>(pushee, normalized_contact));
    BOOST_ASSERT(result.second);
  }
  return pusher_contacts;
}

/* check if two pushees have contact while connected */
void Quasistatic_World_Plugin::check_interpushee(physics::ModelPtr pushee){
  std::vector<physics::Contact*> contacts = contact_manager_->GetContacts();
  for(int i = 0; i < contacts.size(); i++){
    physics::Collision* collision1 = contacts.at(i)->collision1;
    physics::Collision* collision2 = contacts.at(i)->collision2;
    physics::ModelPtr m1 = collision1->GetModel();
    physics::ModelPtr m2 = collision2->GetModel();
    if((cur_pushee == m1 && pusher_ != m2) || 
       (cur_pushee == m2 && pusher_ != m1))
      std::cout << "warning multi-body-collision\n";
  }
}

/* Disable ode physics on a model */
void Quasistatic_World_Plugin::set_ode(physics::ModelPtr m, bool on){
  physics::Link_V links = m->GetLinks();
  for(int i = 0; i < links.size(); i++)
  {
    physics::LinkPtr link = links.at(i);
    if(on)
    {
      link->SetCollideMode("all");
      link->SetGravityMode(false);
    }
    else
    {
      link->SetCollideMode("ghost");
      link->SetGravityMode(false);
    }

  }
}
