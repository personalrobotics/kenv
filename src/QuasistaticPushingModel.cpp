#include <iostream>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <kenv/eigen_yaml.h>
#include "QuasistaticPushingModel.h"

inline Eigen::Vector3d to3D(Eigen::Vector2d const &x)
{
    return (Eigen::Vector3d() << x, 0).finished();
}

namespace quasistatic_pushing {

/*
 * Action
 */
Action::Action()
    : linear_velocity_(0, 0)
    , angular_velocity_(0)
{}

Action::Action(Eigen::Vector2d const &linear_displacement, double const &angular_displacement, double t)
    : t_(t)
{
    angular_velocity_ = angular_displacement/t;
    linear_velocity_ = linear_displacement/t;
}

void Action::apply(Eigen::Affine3d &pose) const
{
    pose.pretranslate(Eigen::Vector3d(linear_velocity_[0]*t_, linear_velocity_[1]*t_, 0));
    pose.prerotate(Eigen::AngleAxisd(angular_velocity_*t_, Eigen::Vector3d::UnitZ()));
}

void Action::apply(kenv::Object::Ptr target) const
{
    Eigen::Affine3d pose = target->getTransform();
    apply(pose);
    target->setTransform(pose);
}

Eigen::Vector2d Action::linear_velocity() const
{
    return linear_velocity_;
}

Eigen::Vector2d Action::linear_displacement() const
{
    return linear_velocity_*t_;
}
double Action::angular_displacement() const
{
    return angular_velocity_*t_;
}

void Action::set_linear_velocity(Eigen::Vector2d const &linear_velocity)
{
    linear_velocity_ = linear_velocity;
}

void Action::set_time(double t)
{
    t_ = t;
}
double Action::get_time()
{
    return t_;
}

double Action::angular_velocity() const
{
    return angular_velocity_;
}

void Action::set_angular_velocity(double angular_velocity)
{
    angular_velocity_ = angular_velocity;
}

Action &Action::operator*=(double scale)
{
    linear_velocity_ *= scale;
    angular_velocity_ *= scale;
    return *this;
}

Action &Action::operator/=(double scale)
{
    BOOST_ASSERT(scale != 0);
    linear_velocity_ /  scale;
    angular_velocity_ /= scale;
    return *this;
}

Action operator*(Action const &action, double scale)
{
    return Action(action) *= scale;
}

Action operator*(double scale, Action const &action)
{
    return Action(action) *= scale;
}

Action operator/(Action const &action, double scale)
{
    return Action(action) /= scale;
}

/*
 * QuasistaticPushingModel
 */
QuasistaticPushingModel::QuasistaticPushingModel(kenv::CollisionChecker::Ptr collision_checker,
                                                 double step_meters, double step_radians)
    : collision_checker_(collision_checker)
    , step_meters_(step_meters)
    , step_radians_(step_radians)
    , max_iterations_(25)
    , epsilon_(1e-6)
{
    this->in_contact = false;
    BOOST_ASSERT(step_meters > 0 && step_radians > 0);
}

void QuasistaticPushingModel::Simulate_Step(kenv::Object::Ptr pusher, kenv::Object::Ptr pushee,
                                       Action const &action, double mu, double c,bool reset) 
{
    
    double const linear_scale = action.linear_displacement().norm() / step_meters_;
    double const angular_scale =  action.angular_displacement() / step_radians_;
    double const scale = std::max(linear_scale, angular_scale);
    int const num_steps = static_cast<int>(scale + 0.5);
    Action const step = action * (1.0 / scale);
    if(reset || step_>num_steps)
      step_ = 0;
    if(step_ < num_steps)
    {
        Eigen::Affine3d original_pose = pushee->getTransform();

        // Push the object until it leaves contact with the hand.
        if(in_contact && iteration_ < max_iterations_)
        {
            in_contact = pushObject(pusher, pushee, step, mu, c);
    
        } else {
        
            // Give up and translate the object with the hand.
            // FIXME: This is broken with non-zero angular velocities.
            if (iteration_ == max_iterations_) {
                original_pose.pretranslate(to3D(step.linear_displacement()));
                pushee->setTransform(original_pose);
            }
            MoveHand(pusher, step);    
        }
    }
    

}
void QuasistaticPushingModel::Simulate(kenv::Object::Ptr pusher, kenv::Object::Ptr pushee,
                                       Action const &action, double mu, double c) const
{
   double const linear_scale = action.linear_displacement().norm() / step_meters_;
    double const angular_scale =  action.angular_displacement() / step_radians_;
    double const scale = std::max(linear_scale, angular_scale);
    int const num_steps = static_cast<int>(scale + 0.5);
    Action const step = action * (1.0 / scale);
    kenv::Environment::Ptr env = pusher->getEnvironment(); 
    for (int i = 0; i < num_steps; ++i) {
        Eigen::Affine3d original_pose = pushee->getTransform();

        // Push the object until it leaves contact with the hand.
        bool in_contact = true;
        size_t iteration;
        for (iteration = 0; in_contact && iteration < max_iterations_; ++iteration) {
            in_contact = pushObject(pusher, pushee, step, mu, c);
            env->runWorld(1);        
        }

        // Give up and translate the object with the hand.
        if (iteration == max_iterations_) {
            original_pose.pretranslate(to3D(step.linear_displacement()));
            pushee->setTransform(original_pose);
        }
        MoveHand(pusher, step);
        env->runWorld(1);
    } 
}
void QuasistaticPushingModel::MoveHand(kenv::Object::Ptr hand, Action const &action) const
{
    Eigen::Affine3d hand_pose = hand->getTransform();
    action.apply(hand_pose);
    hand->setTransform(hand_pose);
}

bool QuasistaticPushingModel::pushObject(kenv::Object::Ptr hand, kenv::Object::Ptr object,
                                         Action const &step, double mu, double c) const
{
    // FIXME: This should use the COF frame, not the object frame.
    Eigen::Affine3d const Tenv_cof = object->getTransform();

    // Collision check to find the contacts.
    std::vector<kenv::Contact> contacts;
    collision_checker_->checkCollision(hand, object, &contacts);

    // Use the limit surface to convert each contact into a twist.
    std::vector<Eigen::Vector3d> twists;
    BOOST_FOREACH (kenv::Contact const &contact, contacts) {
        // Transform everything into the center-of-friction frame.
        Eigen::Vector2d const r = (Tenv_cof.inverse() * contact.position).head<2>();
        Eigen::Vector2d const n = (Tenv_cof.inverse().linear() * contact.normal).head<2>();

        // Compute the instantaneous velocity at the contact location from the
        // hand's twist.
        Eigen::Vector2d const vh_linear_world = step.linear_displacement();
        Eigen::Vector2d const vh_linear_hand = Tenv_cof.matrix().inverse().block<2, 2>(0, 0) * vh_linear_world;
        double const omega = step.angular_displacement();
        Eigen::Vector2d const vp = vh_linear_hand + Eigen::Vector2d(r[1] * omega, -r[0] * omega);

        // Compute the twist and transform it back into the world frame.
        Eigen::Vector3d const q_cof = getPushTwist(n, r, vp, mu, c);
        if (q_cof.norm() > epsilon_) {
            Eigen::Vector3d twist;
            twist << Tenv_cof.matrix().block<2, 2>(0, 0) * q_cof.head<2>(), q_cof[2];
            twists.push_back(twist);
        }
    }

    if (twists.empty()) {
        return false;
    }

    // Average the contact twists to get an aggregate twist.
    Eigen::Vector3d mean_twist = Eigen::Vector3d::Zero();
    BOOST_FOREACH (Eigen::Vector3d const &twist, twists) {
        mean_twist += twist;
    }
    mean_twist /= twists.size();

    // Step the object along the twist.
    Eigen::Vector3d const aggregate_twist = scaleTwist(mean_twist);
    Eigen::Affine3d object_pose = object->getTransform();
    Eigen::Vector3d const translation_step(aggregate_twist[0], aggregate_twist[1], 0);
    Eigen::AngleAxisd const rotation_step(aggregate_twist[2], Eigen::Vector3d::UnitZ());
    object_pose.pretranslate(translation_step);
    object_pose.linear() = object_pose.linear() * rotation_step;
    object->setTransform(object_pose);
    return true;
}

Eigen::Vector3d QuasistaticPushingModel::getPushTwist(Eigen::Vector2d const &n, Eigen::Vector2d const &r,
                                                      Eigen::Vector2d const &vp, double mu, double c) const
{
	// Get the edges of the friction cone.
    Eigen::Vector2d fl, fr;
    getFrictionCone(n, mu, &fl, &fr);
    fl.normalize();
    fr.normalize();

	// Generalized velocities caused by the edges of the friction cone.
    Eigen::Vector3d const ql = getTwistFromForce(c, fl, r);
    Eigen::Vector3d const qr = getTwistFromForce(c, fr, r);

	// Motion cone, the velocities experience by the contact point at the
	// edges of the friction cone.
	Eigen::Vector2d const vl = getVelocityFromTwist(ql, r);
	Eigen::Vector2d const vr = getVelocityFromTwist(qr, r);

	// Check whether the pushing direction is inside the motion cone.
    ContactMode::Enum const mode = getContactMode(vp, n, vl, vr);
    switch (mode) {
    case ContactMode::Sticking: {
        double const c2 = std::pow(c, 2);
		double const x2 = std::pow(r[0], 2);
		double const y2 = std::pow(r[1], 2);
        Eigen::Vector3d q;
		q << ((c2 + x2) * vp[0] + r[0] * r[1] * vp[1]) / (c2 + x2 + y2),
			 (r[0] * r[1] * vp[0] + (c2 + y2) * vp[1]) / (c2 + x2 + y2),
			 (r[0] * vp[1] - r[1] * vp[0]) / c2;
        return q;
    }

    case ContactMode::SlidingLeft: {
        Eigen::Vector3d twist = (vp.dot(n) / vl.dot(n)) * ql;
		return twist;
    }

    case ContactMode::SlidingRight: {
        Eigen::Vector3d twist = (vp.dot(n) / vr.dot(n)) * qr;
        return twist;
    }

    case ContactMode::Separation:
		return Eigen::Vector3d::Zero();

    default:
        throw std::runtime_error("Unknown contact mode.");
	}
}

void QuasistaticPushingModel::getFrictionCone(Eigen::Vector2d const &normal, double mu,
                                              Eigen::Vector2d *fr_l, Eigen::Vector2d *fr_r) const
{
    BOOST_ASSERT(fr_l && fr_r);
    BOOST_ASSERT(mu >= 0);
	double const alpha = std::atan(mu);
	*fr_l = Eigen::Rotation2Dd(+alpha) * normal;
	*fr_r = Eigen::Rotation2Dd(-alpha) * normal;
}

Eigen::Vector3d QuasistaticPushingModel::getTwistFromForce(double c, Eigen::Vector2d const &f,
                                                                     Eigen::Vector2d const &r) const
{
    double const c_sqr = std::pow(c, 2);
    return Eigen::Vector3d(c_sqr * f[0], c_sqr * f[1], cross(r, f));
}

Eigen::Vector2d QuasistaticPushingModel::getVelocityFromTwist(Eigen::Vector3d const &q,
                                                              Eigen::Vector2d const &r) const
{
	return Eigen::Vector2d(q[0] - q[2] * r[1], q[1] + q[2] * r[0]);
}

ContactMode::Enum QuasistaticPushingModel::getContactMode(Eigen::Vector2d const &vp, Eigen::Vector2d const &n,
                                                          Eigen::Vector2d const &vl, Eigen::Vector2d const &vr) const
{
	Eigen::Vector2d const vl_orthog = Eigen::Rotation2Dd(-M_PI / 2) * vl;
	Eigen::Vector2d const vr_orthog = Eigen::Rotation2Dd(+M_PI / 2) * vr;
	double const vl_span = vl_orthog.dot(vp);
	double const vr_span = vr_orthog.dot(vp);

	if (vl_span > 0 && vr_span > 0) {
        return ContactMode::Sticking;
	} else if (vl_span > 0 && vr_span < 0) {
        if(vr.dot(n) * vp.dot(n) > 0){
            return ContactMode::SlidingRight;
        }
	} else if (vl_span < 0 && vr_span > 0) {
        if(vl.dot(n) * vp.dot(n) > 0){
            return ContactMode::SlidingLeft;
        }
	}
    return ContactMode::Separation;
}

double QuasistaticPushingModel::cross(Eigen::Vector2d const &x1, Eigen::Vector2d const &x2) const
{
	return x1[0] * x2[1] - x1[1] * x2[0];
}

Eigen::Vector3d QuasistaticPushingModel::scaleTwist(Eigen::Vector3d const &q) const
{
    double const distance = q.head<2>().norm();
    if (q.head<2>().norm() < epsilon_ && std::fabs(q[2]) < epsilon_) {
        return  Eigen::Vector3d::Zero();
    } else if (distance * step_radians_ > std::fabs(q[2]) * step_meters_) {
        return q * step_meters_ / distance;
    } else {
        return q * step_radians_ / std::fabs(q[2]);
    }
}

}
