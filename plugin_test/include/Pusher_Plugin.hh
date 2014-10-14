#ifndef _PUSHER_PLUGIN_HH_
#define _PUSHER_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_quasistatic_plugin/Quasistatic_World_Plugin.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
	class Pusher_Plugin : public ModelPlugin
	{
	public:
	  Pusher_Plugin();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      void OnUpdate(const common::UpdateInfo & /*_info*/);
	private: 
	  physics::ModelPtr pusher_;
	  event::ConnectionPtr update_connection_;      

	};
};
#endif 
