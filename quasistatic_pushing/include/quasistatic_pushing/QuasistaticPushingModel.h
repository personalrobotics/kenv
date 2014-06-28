#ifndef QUASISTATICPUSHINGMODEL_H_
#define QUASISTATICPUSHINGMODEL_H_

#include <Eigen/Dense>
#include <kenv/Environment.h>
#include <kenv/CollisionChecker.h>

namespace quasistatic_pushing {

struct ContactMode {
    enum Enum {
        Separation,
        Sticking,
        SlidingLeft,
        SlidingRight
    };
};

/*
 * Action
 */
class Action {
public:
    Action();
    Action(Eigen::Vector2d const &linear_displacement, double const &angular_displacement, double time);

    void apply(Eigen::Affine3d &pose) const;
    void apply(kenv::Object::Ptr target) const;

    Eigen::Vector2d linear_displacement() const;
    double angular_displacement() const;

    Eigen::Vector2d linear_velocity() const;
    void set_linear_velocity(Eigen::Vector2d const &linear_velocity);

    double angular_velocity() const;
    void set_angular_velocity(double angular_velocity);

    double get_time();
    void set_time(double t);

    Action &operator*=(double scale);
    Action &operator/=(double scale);

private:
    Eigen::Vector2d linear_velocity_;
    double angular_velocity_;
    double t_;
};

Action operator*(Action const &action, double scale);
Action operator*(double scale, Action const &action);
Action operator/(Action const &action, double scale);

/*
 * QuasistaticPushingModel
 */
class QuasistaticPushingModel {
public:
    typedef boost::shared_ptr<QuasistaticPushingModel> Ptr;
    typedef boost::shared_ptr<QuasistaticPushingModel const> ConstPtr;

    QuasistaticPushingModel(kenv::CollisionChecker::Ptr collision_checker,
                            double step_meters, double step_radians);
    void Simulate_Step(kenv::Object::Ptr pusher, kenv::Object::Ptr pushee,
          Action const &action, double mu, double c, bool reset);
    void MoveHand(kenv::Object::Ptr hand, Action const &vp) const;
    void Simulate(kenv::Object::Ptr pusher, kenv::Object::Ptr pushee,
          Action const &action, double mu, double c) const;

private:
    kenv::CollisionChecker::Ptr collision_checker_;
    double step_meters_, step_radians_;
    size_t max_iterations_;
    double epsilon_;
    int iteration_;
    int step_;
    bool in_contact;


    bool pushObject(kenv::Object::Ptr hand, kenv::Object::Ptr object,
                    Action const &step, double mu, double c) const;

    Eigen::Vector3d getPushTwist(Eigen::Vector2d const &n, Eigen::Vector2d const &r,
                                 Eigen::Vector2d const &vp, double mu, double c) const;
    void getFrictionCone(Eigen::Vector2d const &normal, double mu,
                         Eigen::Vector2d *fr_l, Eigen::Vector2d *fr_r) const;
    Eigen::Vector3d getTwistFromForce(double c, Eigen::Vector2d const &f, Eigen::Vector2d const &r) const;
    Eigen::Vector2d getVelocityFromTwist(Eigen::Vector3d const &q, Eigen::Vector2d const &r) const;
    ContactMode::Enum getContactMode(Eigen::Vector2d const &vp, Eigen::Vector2d const &n,
                                     Eigen::Vector2d const &vl, Eigen::Vector2d const &vr) const;

    double cross(Eigen::Vector2d const &x1, Eigen::Vector2d const &x2) const;
    Eigen::Vector3d scaleTwist(Eigen::Vector3d const &q) const;
};

}

#endif
