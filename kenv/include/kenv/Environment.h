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
#include <boost/multi_array.hpp>

namespace kenv {

class Environment;
class Object;

typedef Eigen::AlignedBox<double, 3> AlignedBox3d;
typedef boost::shared_ptr<void> Handle;

class Contact {
public:
    Contact() {}
    Contact(Eigen::Vector3d const &position, Eigen::Vector3d const &normal)
        : position(position), normal(normal) {}

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

template <typename Scalar>
Contact operator*(Eigen::Transform<Scalar, 3, Eigen::Affine> const &transform,
                  Contact const &contact)
{
    return Contact(transform * contact.position, transform.linear() * contact.normal);
}

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
    virtual void enable(bool flag) = 0;

    virtual AlignedBox3d computeLocalAABB() = 0;
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
                                std::vector<std::pair<Link::Ptr, Link::Ptr> > *links = NULL) const = 0;

    virtual std::vector<Link::Ptr> getLinks(void) const = 0;
    virtual Link::Ptr getLink(std::string const name) const = 0;

    virtual Eigen::Affine3d getTransform(void) const = 0;
    virtual void setTransform(Eigen::Affine3d const &tf) = 0;

    virtual Eigen::VectorXd getDOFValues(void) const = 0;
    virtual void setDOFValues(Eigen::VectorXd const &dof_values) = 0;
    
    virtual void setColor(Eigen::Vector4d const &color) = 0;
    virtual void setTransparency(double p) = 0;
};

class Environment : private boost::noncopyable {
public:
    typedef boost::shared_ptr<Environment> Ptr;
    typedef boost::shared_ptr<Environment const> ConstPtr;

    virtual Object::Ptr getObject(std::string const &name) = 0;
    virtual Object::Ptr createObject(std::string const &type, std::string const &name, bool anonymous = false) = 0;
    virtual void remove(Object::Ptr object) = 0;
    virtual void runWorld(int steps) = 0;

    virtual Handle drawLine(Eigen::Vector3d const &start, Eigen::Vector3d const &end,
                            double width, Eigen::Vector4d const &color) = 0;
    virtual Handle drawLineStrip(std::vector<Eigen::Vector3d> const &points,
                                 double width, Eigen::Vector4d const &color) = 0;
    virtual Handle drawLineList(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > const &lines,
                                double width, Eigen::Vector4d const &color) = 0;
    virtual Handle drawArrow(Eigen::Vector3d const &start, Eigen::Vector3d const &end,
                             double width, Eigen::Vector4d const &color) = 0;
    virtual Handle drawPoints(std::vector<Eigen::Vector3d> const &points,
                              float point_size, Eigen::Vector4d const &color) = 0;

    virtual Handle drawPlane( const Eigen::Affine3d& origin, float width, float height,
    						const boost::multi_array<float,3>& texture)=0;
};

}

#endif
