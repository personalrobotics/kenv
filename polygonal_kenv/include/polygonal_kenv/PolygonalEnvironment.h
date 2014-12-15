#ifndef POLYGONALOBJECT_H_
#define POLYGONALOBJECT_H_

#include <kenv/Environment.h>
#include <geos/geom/CoordinateSequenceFactory.h>
#include <geos/geom/MultiPoint.h>
#include "PolygonalLink.h"

namespace kenv {

class PolygonalEnvironment;

class ColoredGeometry {
public:
    typedef boost::shared_ptr<ColoredGeometry> Ptr;
    typedef boost::shared_ptr<ColoredGeometry const> ConstPtr;
    typedef boost::weak_ptr<ColoredGeometry> WeakPtr;
    typedef boost::weak_ptr<ColoredGeometry const> WeakConstPtr;

    ColoredGeometry(geos::geom::Geometry *geom, Eigen::Vector4d const &color);
    virtual ~ColoredGeometry();

    geos::geom::Geometry *geom;
    Eigen::Vector4d color;
};

class TexturePatch {
public:
    typedef boost::shared_ptr<TexturePatch> Ptr;
    typedef boost::shared_ptr<TexturePatch const> ConstPtr;
    typedef boost::weak_ptr<TexturePatch> WeakPtr;
    typedef boost::weak_ptr<TexturePatch const> WeakConstPtr;

    TexturePatch(Eigen::Affine2d const &origin, double width, double height,
                 boost::multi_array<float, 3> const &texture);

    Eigen::Affine2d origin;
    double width, height;
    boost::multi_array<float, 3> texture;
};

class PolygonalLink : public virtual kenv::Link {
public:
    typedef boost::shared_ptr<PolygonalLink> Ptr;
    typedef boost::shared_ptr<PolygonalLink const> ConstPtr;

    PolygonalLink(::PolygonalLink::Ptr internal_link);
    virtual boost::shared_ptr<Object> getObject(void) const;

    virtual std::string getName(void) const;
    virtual Eigen::Affine3d getTransform(void) const;
    virtual void enable(bool flag);

    virtual boost::shared_ptr<geos::geom::Geometry const> getGeometry() const;

    virtual AlignedBox3d computeLocalAABB();

private:
    bool enable_;
    ::PolygonalLink::Ptr internal_link_;
};

typedef PolygonalLink PolygonalLink_ext;

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

    virtual void saveState();
    virtual void restoreState();

    virtual void enable(bool flag);
    virtual void setVisible(bool flag);
    virtual bool getVisible() const;

    virtual AlignedBox3d getAABB(void) const;
    virtual bool checkCollision(Object::ConstPtr entity, std::vector<Contact> *contacts = NULL,
                                std::vector<std::pair<Link::Ptr, Link::Ptr> > *links = NULL) const;
    virtual bool checkCollision(std::vector<Contact> *contacts = NULL, 
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
    virtual boost::shared_ptr<geos::geom::Geometry const> getGeometry() const;

private:
    std::string type_;
    std::string name_;
    bool visible_;
    double transparency_;
    Eigen::Vector4d color_;
    boost::weak_ptr<PolygonalEnvironment> environment_;
    ::PolygonalLink::Ptr base_link_;
    std::vector<PolygonalLink::Ptr> links_;
    std::vector< ::PolygonalJoint::Ptr> joints_;
    mutable boost::shared_ptr<geos::geom::Geometry> cached_geometry_;
};

class PolygonalEnvironment : public boost::enable_shared_from_this<PolygonalEnvironment>,
                             public virtual kenv::Environment {
public:
    typedef boost::shared_ptr<PolygonalEnvironment> Ptr;
    typedef boost::shared_ptr<PolygonalEnvironment const> ConstPtr;
    typedef boost::shared_ptr<void> Handle;

    PolygonalEnvironment();

    virtual Object::Ptr getObject(std::string const &name);
    virtual void getObjects(std::vector<Object::Ptr>& objects);
    virtual std::vector<Object::Ptr> getObjects() const;
    virtual Object::Ptr createObject(std::string const &type, std::string const &name, bool anonymous = false);

    virtual Robot::Ptr getRobot(std::string const &name);
    virtual Robot::Ptr createRobot(std::string const &type, std::string const &name, bool anonymous = false);

    virtual void remove(Object::Ptr object);
    virtual void runWorld(int steps);
    
    virtual boost::recursive_try_mutex& getMutex();
    
    virtual void saveFullState();
    virtual void restoreFullState();

    virtual bool checkCollision(Object::ConstPtr entity, std::vector<Contact> *contacts = NULL) const;
    virtual bool checkCollision(Object::ConstPtr obj1, Object::ConstPtr obj2, std::vector<Contact> *contacts = NULL) const;
    virtual bool checkCollision(Object::ConstPtr entity, const std::vector<Object::ConstPtr> &objects, std::vector<std::vector<Contact> > *contacts = NULL) const;

    virtual Handle drawGeometry(geos::geom::Geometry *geom, Eigen::Vector4d const &color);
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

    std::vector<ColoredGeometry::Ptr> getVisualizationGeometry();
    std::vector<TexturePatch::Ptr> getTexturePatches();

private:
    std::map<std::string, PolygonalObject::Ptr> objects_;
    geos::geom::GeometryFactory const *geom_factory_;
    geos::geom::CoordinateSequenceFactory const *coords_factory_;
    std::vector<ColoredGeometry::WeakPtr> visualization_;
    std::vector<TexturePatch::WeakPtr> textures_;

    geos::geom::Coordinate toGeos2D(Eigen::Vector3d const &point) const;
};

}

#endif
