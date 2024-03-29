#ifndef ORENVIRONMENT_H_
#define ORENVIRONMENT_H_

#include <boost/enable_shared_from_this.hpp>
#include <boost/filesystem.hpp>
#include <openrave/openrave.h>
#include <kenv/Environment.h>

namespace kenv {

class OREnvironment;
class ORObject;

class ORViewer : private boost::noncopyable {
public:
    typedef boost::shared_ptr<ORViewer> Ptr;
    typedef boost::shared_ptr<ORViewer const> ConstPtr;

    ORViewer(OpenRAVE::EnvironmentBasePtr or_env);
    bool isOpen(void) const;
    void waitForClose(void);
    void redraw(void);

private:
    OpenRAVE::ViewerBasePtr or_viewer_;
    boost::thread thread_;
    bool is_running_;

    void mainLoop(OpenRAVE::EnvironmentBasePtr or_env);
};

class ORLogger : public Logger {
public:
    virtual void debug(std::string const &msg);
    virtual void info(std::string const &msg);
    virtual void warning(std::string const &msg);
    virtual void error(std::string const &msg);
    virtual void fatal(std::string const &msg);
};

class ORLink : virtual public Link {
public:
    typedef boost::shared_ptr<ORLink> Ptr;
    typedef boost::shared_ptr<ORLink const> ConstPtr;

    ORLink(boost::weak_ptr<ORObject> robot, OpenRAVE::KinBody::LinkPtr link);
    virtual Object::Ptr getObject(void) const;

    virtual std::string getName(void) const;
    virtual Eigen::Affine3d getTransform(void) const;
    virtual void enable(bool flag);

    AlignedBox3d computeLocalAABB();

private:
    boost::weak_ptr<ORObject> object_;
    OpenRAVE::KinBody::LinkPtr link_;
};

class ORObject : virtual public Object, public boost::enable_shared_from_this<ORObject> {
public:
    typedef boost::shared_ptr<ORObject> Ptr;
    typedef boost::shared_ptr<ORObject const> ConstPtr;

    ORObject(boost::weak_ptr<OREnvironment> parent, OpenRAVE::KinBodyPtr kinbody,
             std::string const &type);
    virtual void initialize(void);
    OpenRAVE::KinBodyPtr getKinBody(void) const;

    virtual Environment::Ptr getEnvironment(void) const;
    virtual std::string getName(void) const;
    virtual std::string getType(void) const;
    virtual std::string getKinematicsGeometryHash(void) const;

    virtual bool checkCollision(Object::ConstPtr entity, std::vector<Contact> *contacts = NULL,
                                std::vector<std::pair<Link::Ptr, Link::Ptr> > *links = NULL) const;

    virtual std::vector<Link::Ptr> getLinks(void) const;
    virtual Link::Ptr getLink(std::string const name) const;

    virtual void enable(bool flag);
    virtual void setVisible(bool flag);

    virtual Eigen::Affine3d getTransform(void) const;
    virtual void setTransform(Eigen::Affine3d const &tf);
    virtual AlignedBox3d getAABB(void) const;

    virtual Eigen::VectorXd getDOFValues(void) const;
    virtual void setDOFValues(Eigen::VectorXd const &dof_values);

    virtual void setColor(Eigen::Vector4d const &color);
    virtual void setTransparency(double p);

private:
    boost::weak_ptr<OREnvironment> parent_;
    OpenRAVE::KinBodyPtr kinbody_;
    std::map<std::string, ORLink::Ptr> links_;
    std::string type_;

    OpenRAVE::CollisionAction checkCollisionCallback(OpenRAVE::CollisionReportPtr report, bool is_physics,
                                                     std::vector<std::pair<Link::Ptr, Link::Ptr> > *links) const;
};

class OREnvironment : public Environment, public boost::enable_shared_from_this<OREnvironment> {
public:
    typedef boost::shared_ptr<OREnvironment> Ptr;
    typedef boost::shared_ptr<OREnvironment const> ConstPtr;

    OREnvironment(void);
    OREnvironment(OpenRAVE::EnvironmentBasePtr or_env);
    OpenRAVE::EnvironmentBasePtr getOREnvironment(void) const;
    virtual Object::Ptr getObject(std::string const &name);

    virtual void runWorld(int);

    virtual Object::Ptr createObject(std::string const &type, std::string const &name, bool anonymous = false);
    virtual void remove(Object::Ptr object);

    virtual Handle drawLine(Eigen::Vector3d const &start, Eigen::Vector3d const &end,
                            double width, Eigen::Vector4d const &color);
    virtual Handle drawLineList(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > const &lines,
                                double width, Eigen::Vector4d const &color);
    virtual Handle drawLineStrip(std::vector<Eigen::Vector3d> const &points,
                                 double width, Eigen::Vector4d const &color);
    virtual Handle drawArrow(Eigen::Vector3d const &start, Eigen::Vector3d const &end,
                             double width, Eigen::Vector4d const &color);
    virtual Handle drawPoints(std::vector<Eigen::Vector3d> const &points,
                              float point_size, Eigen::Vector4d const &color);

    virtual Handle drawPlane( const Eigen::Affine3d& origin, float width, float height,
    						const boost::multi_array<float,3>& texture);

    void addType(std::string const &type, std::string const &kinbody_path);

    ORViewer::Ptr attachViewer(void);

private:
    OpenRAVE::EnvironmentBasePtr env_;
    std::map<std::string, std::string> types_;
    std::map<OpenRAVE::KinBodyPtr, ORObject::Ptr> objects_;

    bool addObject(ORObject::Ptr object, std::string const name, bool anonymous);
};

}

#endif
