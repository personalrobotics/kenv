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

// TODO: This should be in a namespace.
struct Pushee {
  double mu, c;
};

namespace gazebo
{
  class Quasistatic_World_Plugin : public WorldPlugin
  {
    public:
      Quasistatic_World_Plugin();
      void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
      void OnUpdate(const common::UpdateInfo & /*_info*/);

    private:
      boost::unordered_map<physics::ModelPtr, physics::Contact> get_pusher_contacts();
      void check_interpushee(physics::ModelPtr pushee);
      void set_ode(physics::ModelPtr m, bool on);

      inline void set_pusher(physics::ModelPtr pusher)
      {
        pusher_ = pusher;
      }

      inline void add_pushee(physics::ModelPtr model, double mu, double c)
      {
        BOOST_ASSERT(model);

        Pushee pushee;
        pushee.mu = mu;
        pushee.c = c;

        auto result = pushees_.insert(std::make_pair(model, pushee));
        if (!result.second) {
          throw std::runtime_error(boost::str(boost::format("Duplicate model '%s'.") % model->GetName()));
        }
      }
      
      /* ROS Stuff */
      bool has_contact_;

      physics::ModelPtr pusher_;
      boost::unordered_map<physics::ModelPtr, Pushee> pushees_;
      
      /* Pusher Initial Conditions */
      // TODO: This set should store weak pointers.
      std::set<physics::ModelPtr> active_models_;
      double pusher_vel_[3];
      
      /* Gazebo entities */
      event::ConnectionPtr update_connection_;      
      physics::WorldPtr world;
      physics::PhysicsEnginePtr engine;
      physics::ContactManager* contact_manager_;
      physics::Model_V pushee_pool_;
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