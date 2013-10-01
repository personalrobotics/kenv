#ifndef POLYGONALVIEWER_H_
#define POLYGONALVIEWER_H_

#include <set>
#include <string>
#include <boost/optional.hpp>
#include <Eigen/Dense>
#include <geos/geom/Point.h>
#include <geos/geom/Geometry.h>
#include <geos/geom/GeometryCollection.h>
#include <geos/geom/LineString.h>
#include <geos/geom/Polygon.h>
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include "PolygonalEnvironment.h"

namespace kenv {

class PolygonalViewer {
public:
    typedef boost::shared_ptr<PolygonalViewer> Ptr;
    typedef boost::shared_ptr<PolygonalViewer const> ConstPtr;

    PolygonalViewer(kenv::PolygonalEnvironment::Ptr env, std::string const &name,
                    size_t const width = 800, size_t const height = 600,
                    double const scale = 1000,
                    Eigen::Vector2d const &origin = Eigen::Vector2d::Zero());

    bool Screenshot(std::string const &path);

    void Spin();
    void SpinOnce();
    void Redraw();

    void Select(sf::Vector2f const &point_screen);
    void Drag(sf::Vector2f const &cursor_curr);

private:
    kenv::PolygonalEnvironment::Ptr env_;
    boost::shared_ptr<sf::RenderWindow> window_;
    double scale_;
    Eigen::Vector2d origin_;

    std::set<kenv::Object::Ptr> selection_;
    bool dragging_;
    sf::Vector2f cursor_prev_;

    sf::Color background_color_;
    double outline_thickness_;

    void FromGeometry(geos::geom::Geometry const *geom, std::vector<sf::Drawable *> &shapes,
                      boost::optional<sf::Color> color = boost::optional<sf::Color>());
    void FromPolygon(geos::geom::Polygon const *polygon, std::vector<sf::Drawable *> &shapes,
                     boost::optional<sf::Color> color);
    void FromGeometryCollection(geos::geom::GeometryCollection const *multipolygon,
                                std::vector<sf::Drawable *> &shapes,
                                boost::optional<sf::Color> color);
    void FromLineString(geos::geom::LineString const *linestring, std::vector<sf::Drawable *> &shapes,
                        boost::optional<sf::Color> color);

    sf::Vector2f Project(geos::geom::Coordinate const &coord) const;
    geos::geom::Point *Reproject(sf::Vector2f const &p) const;
    sf::Color toSFMLColor(Eigen::Vector4d const &color) const;
};

}

#endif
