#ifndef ENVIRONMENT_H_
#define ENVIRONMENT_H_

#include <map>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>
#include <boost/serialization/access.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "eigen_serialization.h"

namespace kenv {

class Environment;
class Robot;
class Object;
typedef Eigen::AlignedBox<double, 3> AlignedBox3d;

class Contact {
public:
    Eigen::Vector3d position;
    Eigen::Vector3d normal;

private:
    template <class Archive>
    void serialize(Archive &archive, unsigned int const version) {
        archive & position;
        archive & normal;
    }

    friend class boost::serialization::access;
};

class Logger : private boost::noncopyable {
public:
    virtual void debug(std::string const &msg) = 0;
    virtual void info(std::string const &msg) = 0;
    virtual void warning(std::string const &msg) = 0;
    virtual void error(std::string const &msg) = 0;
    virtual void fatal(std::string const &msg) = 0;

    template<typename T> static void initialize(void)
    {
        BOOST_ASSERT(!instance_);
        instance_ = new T;
    }

    static Logger &getInstance(void)
    {
        BOOST_ASSERT(instance_);
        return *instance_;
    }

private:
    static Logger *instance_;
};

#define LOG_DEBUG(_x_) Logger::getInstance().debug(_x_)
#define LOG_INFO(_x_) Logger::getInstance().info(_x_)
#define LOG_WARNING(_x_) Logger::getInstance().warning(_x_)
#define LOG_ERROR(_x_) Logger::getInstance().error(_x_)
#define LOG_FATAL(_x_) Logger::getInstance().fatal(_x_)

class Link : private boost::noncopyable {
public:
    typedef boost::shared_ptr<Link> Ptr;
    typedef boost::shared_ptr<Link const> ConstPtr;

    virtual boost::shared_ptr<Object> getObject(void) const = 0;

    virtual std::string getName(void) const = 0;
    virtual Eigen::Affine3d getTransform(void) const = 0;
};

class Object : private boost::noncopyable {
public:
    typedef boost::shared_ptr<Object> Ptr;
    typedef boost::shared_ptr<Object const> ConstPtr;

    virtual boost::shared_ptr<Environment> getEnvironment(void) const = 0;
    virtual std::string getName(void) const = 0;
    virtual std::string getType(void) const = 0;
    virtual std::string getKinematicsGeometryHash(void) const = 0;

    virtual void enable(bool flag) = 0;
    virtual void setVisible(bool flag) = 0;
    virtual AlignedBox3d getAABB(void) const = 0;
    virtual bool checkCollision(Object::ConstPtr entity, std::vector<Contact> *contacts = NULL,
                                Link::Ptr *link1 = NULL, Link::Ptr *link2 = NULL) const = 0;

    virtual std::vector<Link::Ptr> getLinks(void) const = 0;
    virtual Link::Ptr getLink(std::string const name) const = 0;

    virtual Eigen::Affine3d getTransform(void) const = 0;
    virtual void setTransform(Eigen::Affine3d const &tf) = 0;

    virtual Eigen::VectorXd getDOFValues(void) const = 0;
    virtual void setDOFValues(Eigen::VectorXd const &dof_values) = 0;
    
    virtual void setColor(Eigen::Vector4d const &color) = 0;
    virtual void setTransparency(double p) = 0;
};

class Robot : virtual public Object {
public:
    typedef boost::shared_ptr<Robot> Ptr;
    typedef boost::shared_ptr<Robot const> ConstPtr;
};

class Environment : private boost::noncopyable {
public:
    typedef boost::shared_ptr<Environment> Ptr;
    typedef boost::shared_ptr<Environment const> ConstPtr;
    typedef boost::shared_ptr<void> Handle;

    virtual Object::Ptr getObject(std::string const &name) = 0;
    virtual Robot::Ptr getRobot(std::string const &name) = 0;

    virtual Object::Ptr createObject(std::string const &type, std::string const &name, bool anonymous = false) = 0;
    virtual Robot::Ptr createRobot(std::string const &type, std::string const name, bool anonymous = false) = 0;
    virtual void remove(Object::Ptr object) = 0;

    virtual Handle drawLine(Eigen::Vector3d const &start, Eigen::Vector3d const &end,
                            double width, Eigen::Vector4d const &color) = 0;
    virtual Handle drawLineStrip(std::vector<Eigen::Vector3d> const &points,
                                 double width, Eigen::Vector4d const &color) = 0;
    virtual Handle drawArrow(Eigen::Vector3d const &start, Eigen::Vector3d const &end,
                             double width, Eigen::Vector4d const &color) = 0;
    virtual Handle drawPoints(std::vector<Eigen::Vector3d> const &points,
                              float point_size, Eigen::Vector4d const &color) = 0;
};

}

#endif
