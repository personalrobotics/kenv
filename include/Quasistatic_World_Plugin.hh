#ifndef _GAZEBO_QUASISTATIC_WORLD_PLUGIN_HH_
#define _GAZEBO_QUASISTATIC_WORLD_PLUGIN_HH_

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "sdf/sdf.hh"
#include "ros/ros.h"
#include "gz_kenv/gz_kenv.h"
#include "kenv/Environment.h"
#include "quasistatic_pushing/QuasistaticPushingModel.h"

namespace gazebo
{
  class Quasistatic_World_Plugin : public WorldPlugin
  {
    public:
      Quasistatic_World_Plugin();
      void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
      void OnUpdate(const common::UpdateInfo & /*_info*/);
  
    private:
      void load_objects();
      void load_object(std::string filename);
      void check_interpushee(physics::ModelPtr pushee);
      bool check_pusher_contact();
      void set_ode(physics::ModelPtr m, bool on);
      

      /* ROS Stuff */
      bool has_contact_;
      
      /* Pusher Initial Conditions */
      double pusher_vel_[3];
      
      /* Gazebo entities */
      event::ConnectionPtr update_connection_;      
      physics::WorldPtr world;
      physics::PhysicsEnginePtr engine;
      physics::ContactManager* contact_manager_;
      physics::Model_V pushee_pool_;
      physics::ModelPtr pusher;
      physics::ModelPtr cur_pushee;


      /* Kenvs */
      kenv::GazeboEnvironment::Ptr kenv_world;

      /* Quasistatic Pushing */
      quasistatic_pushing::QuasistaticPushingModel::Ptr simulator_;

      bool objects_loaded_;
      bool cur_pushee_set;
  };
};
#endif
