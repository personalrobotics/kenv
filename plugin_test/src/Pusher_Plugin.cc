#include "Pusher_Plugin.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "Quasistatic_World_Plugin.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/transport/transport.hh" 
#include "gazebo/transport/transport.hh"
#include "sdf/sdf.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(Pusher_Plugin)

Pusher_Plugin::Pusher_Plugin()
{}

void Pusher_Plugin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*sdf*/)
{
	this->pusher_ = _parent;
	  /* Listen to 'step' events */
  	this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&Pusher_Plugin::OnUpdate, this, _1));
}

void Pusher_Plugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{
	pusher_->SetLinearVel(math::Vector3(0.03,0,0));
	pusher_->SetAngularVel(math::Vector3(0,0,0));
	physics::Joint_V joints = pusher_ -> GetJoints();
	physics::Link_V links = pusher_ -> GetLinks();
	for(int i = 0; i < pusher_->GetJointCount(); i++){
		physics::JointPtr j = joints.at(i);
		j->Reset();
		j->SetStiffness(0, 100);
 	}
	for(int i = 0; i < links.size(); i++){
		physics::LinkPtr l = links.at(i);
	}
}


