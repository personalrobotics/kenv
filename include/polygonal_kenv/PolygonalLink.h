#ifndef POLYGONALLINK_H_
#define POLYGONALLINK_H_

#include <vector>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <geos/geom/MultiPolygon.h>
#include <yaml-cpp/yaml.h>

geos::geom::Geometry *TransformGeometry(geos::geom::Geometry *multipolygon,
                                        Eigen::Affine2d const &transform);

class PolygonalJoint;

class PolygonalLink : public boost::enable_shared_from_this<PolygonalLink> {
public:
    typedef boost::shared_ptr<PolygonalLink> Ptr;
    typedef boost::weak_ptr<PolygonalLink> WeakPtr;
    typedef boost::shared_ptr<PolygonalLink const> ConstPtr;
    typedef boost::weak_ptr<PolygonalLink const> ConstWeakPtr;

    PolygonalLink();
    PolygonalLink(std::string const &name, Eigen::Affine2d const &relative_pose,
                  geos::geom::Geometry *geometry);
    ~PolygonalLink();
    void update();

    Eigen::Affine2d pose() const;
    Eigen::Affine2d relative_pose() const;
    void set_pose(Eigen::Affine2d const &pose);

    std::string name() const;
    std::vector<boost::shared_ptr<PolygonalJoint> > joints() const;
    std::vector<PolygonalLink::Ptr> children() const;
    boost::shared_ptr<geos::geom::Geometry const> geometry() const;

    boost::shared_ptr<PolygonalJoint> AddJoint(std::string const &name, PolygonalLink::Ptr child_link,
                                               Eigen::Vector2d const &origin, int direction);

    void deserialize(YAML::Node const &node);
    void serialize(YAML::Emitter &emitter) const;

private:
    std::string name_;
    Eigen::Affine2d pose_;
    Eigen::Affine2d relative_pose_;
    std::vector<boost::shared_ptr<PolygonalJoint> > joints_;

    boost::shared_ptr<geos::geom::Geometry> relative_geometry_;
    mutable boost::shared_ptr<geos::geom::Geometry> absolute_geometry_;

    friend class PolygonalJoint;
};

class PolygonalJoint : public boost::enable_shared_from_this<PolygonalJoint> {
public:
    typedef boost::shared_ptr<PolygonalJoint> Ptr;
    typedef boost::shared_ptr<PolygonalJoint const> ConstPtr;

    PolygonalJoint();
    PolygonalJoint(std::string const &name,
                   PolygonalLink::Ptr parent_link, PolygonalLink::Ptr Child_link,
                   Eigen::Vector2d const &origin, int sign);
    void update();

    std::string name() const;
    Eigen::Affine2d origin() const;
    PolygonalLink::Ptr parent() const;
    PolygonalLink::Ptr child() const;

    double angle() const;
    void set_angle(double angle);

    void deserialize(PolygonalLink::Ptr parent_link, YAML::Node const &node);
    void serialize(YAML::Emitter &emitter) const;

private:
    std::string name_;
    PolygonalLink::WeakPtr parent_link_;
    PolygonalLink::Ptr child_link_;

    Eigen::Affine2d relative_origin_;
    int direction_;
    double angle_;
};

YAML::Emitter &operator<<(YAML::Emitter &emitter, PolygonalLink const &link);
YAML::Emitter &operator<<(YAML::Emitter &emitter, PolygonalJoint const &joint);

#endif
