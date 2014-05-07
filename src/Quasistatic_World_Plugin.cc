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

void Quasistatic_World_Plugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf){
  world = _parent;
  kenv_world = boost::make_shared<kenv::GazeboEnvironment>(world);
  
  /* Initialize with active gazebosim and no quasistatic */
  has_contact_ = false;
  objects_loaded_ = false;
  cur_pushee_set = false;
  pusher_vel_[0] = 0.03;
  pusher_vel_[1] = 0;
  pusher_vel_[2] = 0;

  /* Set up phyiscs Engine */
  engine = world->GetPhysicsEngine();
  engine->SetGravity(math::Vector3(0., 0., 0.));
  engine->SetRealTimeUpdateRate(1e9);
  engine->SetMaxStepSize(0.001);

  /* Set up contact manager to get contacts */
  contact_manager_ = engine->GetContactManager();
  load_objects();

  /* Listen to 'step' events */
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&Quasistatic_World_Plugin::OnUpdate, this, _1));
  world->EnableAllModels();
  // world->SetPaused(true);
}

//Note that this implies that that only one object can be updated at a time
void Quasistatic_World_Plugin::OnUpdate(const common::UpdateInfo & /*_info*/){
   // std::cout << "Number of Models loaded" << world->GetModels().size() << '\n';
  if(world->GetModels().size() <= 2)
     return;
  if(!objects_loaded_){
    if(pushee_pool_.size() < world->GetModels().size()-2)
    {
      for(int i = 0; i < world->GetModels().size()-2; i++)
      {
        pushee_pool_.push_back(world->GetModels().at(i+2));
      }
    }
    pusher = world->GetModel("Hand");
      math::Vector3 trans(0,0,.5);
    math::Quaternion rot(0,1,0,cos(M_PI/4));
    math::Pose p(trans,rot);
    pusher->SetRelativePose(p);
    
    objects_loaded_ = true;
  }
  math::Pose p = pusher->GetRelativePose();
  math::Vector3 vp = p.pos;
  math::Vector3 new_pos(vp.x,vp.y, .5);
  math::Quaternion rot(0,1,0,cos(M_PI/4));
  math::Pose np(new_pos, rot);
  pusher->SetRelativePose(np); 
  physics::Joint_V jv = pusher->GetJoints();
  for(unsigned int i = 0; i < pusher->GetJointCount(); i++)
  {
    physics::JointPtr j = jv.at(i);
    j->Reset();
  } 
  has_contact_ = check_pusher_contact(); 

  if(has_contact_){
    //SET UP CONTROLS
     std::cout << "Contact\n";
     double mu = 0.5;
     double c = 0.5;

     math::Vector3 vv = pusher->GetRelativeLinearVel();
     Eigen::Vector2d v(vv.x,vv.y);
     quasistatic_pushing::Action a(v, 0.0,.001);
     simulator_->Simulate_Step(kenv_world->getObject(pusher->GetName()), 
                                    kenv_world->getObject(cur_pushee->GetName()),  a, mu, c, false);

     
     math::Pose pose = pusher->GetRelativePose();
     math::Vector3 trans = pose.pos;
     set_ode(pusher, true);
     set_ode(cur_pushee, true);
     pusher->SetAngularVel(math::Vector3(0,0,0));
     

  } else {
     std::cout << "No Contact\n";
     math::Pose pose = pusher->GetRelativePose();
     math::Vector3 trans = pose.pos;
    if(cur_pushee_set){
      std::cout << "Stop Pushee\n";
       cur_pushee->SetLinearVel(math::Vector3(0,0,0));
       cur_pushee->SetAngularVel(math::Vector3(0,0,0));
    }
    //Comment this out when integrating with state estimator

    pusher->SetLinearVel(math::Vector3(pusher_vel_[0],pusher_vel_[1], pusher_vel_[2]));
    pusher->SetAngularVel(math::Vector3(0,0,0));

    //THIS IS FOR STATE ESTIMATOR
    // pusher->SetLinearVel(pusher->GetRelativeLinearVel());
    // pusher->SetAngularVel(pusher->GetRelativeAngularVel());
  }
  ros::spinOnce();
}

/* check if pusher has contact  */
bool Quasistatic_World_Plugin::check_pusher_contact(){
  // std::cout << "Checking Pusher Contacts\n";
  std::vector<physics::Contact*> contacts = contact_manager_->GetContacts();
  // std::cout << "num contacts " << contacts.size() << '\n';

  for(int i = 0; i < contact_manager_->GetContactCount(); i++){        
    physics::Collision* collision1 = contacts.at(i)->collision1;
    physics::Collision* collision2 = contacts.at(i)->collision2;
    std::string name1 = collision1->GetModel()->GetName();
    std::string name2 = collision2->GetModel()->GetName();
    std::string pusher_name = pusher->GetName();

    std::cout << "Collision: " << name1 << "," << name2 << "\n";
    // std::cout << "Check pusher has collided\n";
    if(name1 == "ground_plane" || 
       name2 == "ground_plane")
    {
      continue;
    }
    if(pusher_name == name1)
    { 

      cur_pushee = collision2->GetModel();
      cur_pushee_set = true;
      return true;
    }
    else if(pusher_name == name2)
    {
      cur_pushee = collision1->GetModel();
      cur_pushee_set = true;
      return true;
    }
    else {}
    // std::cout << "Waiting For Collision\n";


  }
  return false;
}

/* load objects into the evironment */
void Quasistatic_World_Plugin::load_objects(){
  /* TODO get manifest of sdfs */
  
  /* TODO load each sdf into the world  */
  
  /* Test Objects */
  // TODO: Load these objects with kenv.
  load_object("/homes/mkoval/ros/gazebo_push_grasp_sim/models/bh280_standalone.model");
  load_object("/homes/mkoval/ros/gazebo_push_grasp_sim/models/pushee_box.model");
}

void Quasistatic_World_Plugin::load_object(std::string filename){
  std::ifstream model_file;
  model_file.open(filename.c_str());
  
  std::string model_sdf;
  sdf::SDF model;
  model_file.seekg(0, std::ios::end);   
  model_sdf.reserve(model_file.tellg());
  model_file.seekg(0, std::ios::beg);
  
  model_sdf.assign(std::istreambuf_iterator<char>(model_file),
        std::istreambuf_iterator<char>());
        
  model.SetFromString(model_sdf);
  world->InsertModelSDF(model);
}

/* check if two pushees have contact while connected */
void Quasistatic_World_Plugin::check_interpushee(physics::ModelPtr pushee){
  std::vector<physics::Contact*> contacts = contact_manager_->GetContacts();
  for(int i = 0; i < contacts.size(); i++){
    physics::Collision* collision1 = contacts.at(i)->collision1;
    physics::Collision* collision2 = contacts.at(i)->collision2;
    physics::ModelPtr m1 = collision1->GetModel();
    physics::ModelPtr m2 = collision2->GetModel();
    if((cur_pushee == m1 && pusher != m2) || 
       (cur_pushee == m2 && pusher != m1))
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
