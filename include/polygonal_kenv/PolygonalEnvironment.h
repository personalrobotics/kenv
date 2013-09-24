#ifndef POLYGONALOBJECT_H_
#define POLYGONALOBJECT_H_

#include <kenv/Environment.h>
#include "PolygonalLink.h"

namespace kenv {

class PolygonalEnvironment;

class PolygonalLink : public virtual kenv::Link {
public:
    typedef boost::shared_ptr<PolygonalLink> Ptr;
    typedef boost::shared_ptr<PolygonalLink const> ConstPtr;

    PolygonalLink(::PolygonalLink::Ptr internal_link);
    virtual boost::shared_ptr<Object> getObject(void) const;

    virtual std::string getName(void) const;
    virtual Eigen::Affine3d getTransform(void) const;
    virtual void enable(bool flag);

    virtual geos::geom::Geometry *getGeometry() const;
    virtual std::vector<geos::geom::Geometry *> getSensors() const;

    virtual AlignedBox3d computeLocalAABB();

private:
    bool enable_;
    ::PolygonalLink::Ptr internal_link_;
};

class PolygonalObject : public virtual kenv::Object {
public:
    typedef boost::shared_ptr<PolygonalObject> Ptr;
    typedef boost::shared_ptr<PolygonalObject const> ConstPtr;

    PolygonalObject(std::string const &type, std::string const &name,
                    boost::shared_ptr<PolygonalEnvironment> environment,
                    ::PolygonalLink::Ptr base_link);

    virtual boost::shared_ptr<Environment> getEnvironment(void) const;
    virtual std::string getName(void) const;
    virtual std::string getType(void) const;
    virtual std::string getKinematicsGeometryHash(void) const;

    virtual void enable(bool flag);
    virtual void setVisible(bool flag);
    virtual AlignedBox3d getAABB(void) const;
    virtual bool checkCollision(Object::ConstPtr entity, std::vector<Contact> *contacts = NULL,
                                std::vector<std::pair<Link::Ptr, Link::Ptr> > *links = NULL) const;

    virtual std::vector<Link::Ptr> getLinks(void) const;
    virtual Link::Ptr getLink(std::string const name) const;

    virtual Eigen::Affine3d getTransform(void) const;
    virtual void setTransform(Eigen::Affine3d const &tf);

    virtual Eigen::VectorXd getDOFValues(void) const;
    virtual void setDOFValues(Eigen::VectorXd const &dof_values);
    
    virtual void setColor(Eigen::Vector4d const &color);
    virtual Eigen::Vector4d getColor() const;

    virtual void setTransparency(double p);
    virtual double getTransparency() const;

    ::PolygonalLink::Ptr getBaseLink() const;
    virtual geos::geom::Geometry *getGeometry() const;
    virtual std::vector<geos::geom::Geometry *> getSensors() const;

private:
    std::string type_;
    std::string name_;
    double transparency_;
    Eigen::Vector4d color_;
    boost::weak_ptr<PolygonalEnvironment> environment_;
    ::PolygonalLink::Ptr base_link_;
    std::vector<PolygonalLink::Ptr> links_;
    std::vector< ::PolygonalJoint::Ptr> joints_;
};

class PolygonalEnvironment : public boost::enable_shared_from_this<PolygonalEnvironment>,
                             public virtual kenv::Environment {
public:
    typedef boost::shared_ptr<PolygonalEnvironment> Ptr;
    typedef boost::shared_ptr<PolygonalEnvironment const> ConstPtr;
    typedef boost::shared_ptr<void> Handle;

    virtual Object::Ptr getObject(std::string const &name);
    virtual std::vector<Object::Ptr> getObjects() const;
    virtual Object::Ptr createObject(std::string const &type, std::string const &name, bool anonymous = false);
    virtual void remove(Object::Ptr object);

    virtual Handle drawLine(Eigen::Vector3d const &start, Eigen::Vector3d const &end,
                            double width, Eigen::Vector4d const &color);
    virtual Handle drawLineStrip(std::vector<Eigen::Vector3d> const &points,
                                 double width, Eigen::Vector4d const &color);
    virtual Handle drawLineList(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > const &lines,
                                double width, Eigen::Vector4d const &color);
    virtual Handle drawArrow(Eigen::Vector3d const &start, Eigen::Vector3d const &end,
                             double width, Eigen::Vector4d const &color);
    virtual Handle drawPoints(std::vector<Eigen::Vector3d> const &points,
                              float point_size, Eigen::Vector4d const &color);
    virtual Handle drawPlane(Eigen::Affine3d const &origin, float width, float height,
    						boost::multi_array<float,3> const &texture);

private:
    std::map<std::string, PolygonalObject::Ptr> objects_;
};

}

#endif
