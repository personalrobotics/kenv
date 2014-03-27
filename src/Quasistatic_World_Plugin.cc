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
  std::cout << "Setting Up World Plugin\n";
  int argc = 0;
  ros::init(argc,NULL, "Quasistatic_World_Plugin");
  this->collision_checker = boost::make_shared<kenv::DefaultCollisionChecker>();
  simulator = 
    boost::make_shared<quasistatic_pushing::QuasistaticPushingModel>(this->collision_checker, .0001,.0001);
}
void Quasistatic_World_Plugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf){
  this->world = _parent;
  this->kenv_world = boost::make_shared<kenv::GazeboEnvironment>(this->world);

  /* Load Models */
  
  /* Initialize with active gazebosim and no quasistatic */
  has_contact = false;
  simStarted = false;
  objects_loaded = false;
  cur_pushee_set = false;
  pusher_vel[0] = .03;
  pusher_vel[1] = 0;
  pusher_vel[2] = 0;
  /* Set up phyiscs Engine */
  engine = this->world->GetPhysicsEngine();
  engine->SetRealTimeUpdateRate(1000000000);
  engine->SetMaxStepSize(.001);

  /* Set up contact manager to get contacts */
  contact_manager = engine->GetContactManager();
  load_objects();
  /* Listen to 'step' events */
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Quasistatic_World_Plugin::OnUpdate, this, _1));
  world->EnableAllModels();
  engine->SetGravity(math::Vector3(0,0,0));
  world->SetPaused(true);
}


//Note that this implies that that only one object can be updated at a time
void Quasistatic_World_Plugin::OnUpdate(const common::UpdateInfo & /*_info*/){
   // std::cout << "Number of Models loaded" << this->world->GetModels().size() << '\n';
  if(this->world->GetModels().size() <= 2)
     return;
  if(!objects_loaded){
    if(this->pushee_pool.size() < this->world->GetModels().size()-2)
    {
      for(int i = 0; i < this->world->GetModels().size()-2; i++)
      {
        pushee_pool.push_back(this->world->GetModels().at(i+2));
      }
    }
    pusher = this->world->GetModel("pusher_box");
    objects_loaded = true;
  }
    

  has_contact = check_pusher_contact(); 

  if(has_contact){
    //SET UP CONTROLS
     double mu = 0.5;
     double c = 0.5;
     Eigen::Vector2d v(pusher_vel[0],pusher_vel[1]);
     quasistatic_pushing::Action a(v, 0.0);
     simulator->Simulate(kenv_world->getObject(this->pusher->GetName()), 
                                    kenv_world->getObject(this->cur_pushee->GetName()),  a, mu, c);

     
     math::Pose pose = this->pusher->GetRelativePose();
     math::Vector3 trans = pose.pos;
     printf("Quasistatic %f, %f, %f\n", trans.x, trans.y, trans.z);
     set_ode(pusher, true);
     set_ode(cur_pushee, true);
     this->pusher->SetAngularVel(math::Vector3(0,0,0));


  } else {
     math::Pose pose = this->pusher->GetRelativePose();
     math::Vector3 trans = pose.pos;
     printf("Gazebo %f, %f, %f\n", trans.x, trans.y, trans.z);
    if(cur_pushee_set){
      set_ode(pusher, true);
      set_ode(cur_pushee, true);
    }
    this->pusher->SetLinearVel(math::Vector3(pusher_vel[0],pusher_vel[1], pusher_vel[2]));
    this->pusher->SetAngularVel(math::Vector3(0,0,0));
  }
  ros::spinOnce();
}

/* check if pusher has contact  */
bool Quasistatic_World_Plugin::check_pusher_contact(){
  // std::cout << "Checking Pusher Contacts\n";
  std::vector<physics::Contact*> contacts = this->contact_manager->GetContacts();
  // std::cout << "num contacts " << contacts.size() << '\n';
  for(int i = 0; i < contacts.size(); i++){        
    physics::Collision* collision1 = contacts.at(i)->collision1;
    physics::Collision* collision2 = contacts.at(i)->collision2;
    std::string name1 = collision1->GetModel()->GetName();
    std::string name2 = collision2->GetModel()->GetName();
    std::string pusher_name = pusher->GetName();
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
  load_object("../models/pusher_box.model");
  load_object("../models/pushee_box.model");

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
  std::vector<physics::Contact*> contacts = contact_manager->GetContacts();
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
