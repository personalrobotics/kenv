#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/tuple/tuple.hpp>
#include <geos/geom/GeometryFactory.h>
#include "PolygonalViewer.h"

namespace kenv {

PolygonalViewer::PolygonalViewer(kenv::PolygonalEnvironment::Ptr env, std::string const &name,
                                 size_t const width, size_t const height, double const scale,
                                 Eigen::Vector2d const &origin)
    : env_(env)
    , scale_(scale)
    , origin_(origin)
    , dragging_(false)
    , background_color_(0, 0, 0)
    , outline_thickness_(3)
{
    BOOST_ASSERT(env);
    BOOST_ASSERT(scale_ > 0);

    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    window_ = boost::make_shared<sf::RenderWindow>(sf::VideoMode(width, height), name);
    window_->setActive(false);
}

bool PolygonalViewer::Screenshot(std::string const &path)
{
    sf::Image image = window_->capture();
    return image.saveToFile(path);
}

void PolygonalViewer::Spin()
{
    while (window_->isOpen()) {
        SpinOnce();
    }
}

void PolygonalViewer::SpinOnce()
{
    sf::Event event;
    while (window_->pollEvent(event)) {
        switch (event.type) {
        case sf::Event::Closed:
            window_->close();
            break;

        case sf::Event::MouseButtonPressed:
            Select(sf::Vector2f(event.mouseButton.x, event.mouseButton.y));
            dragging_ = true;
            break;

        case sf::Event::MouseButtonReleased:
            dragging_ = false;
            break;

        case sf::Event::MouseMoved:
            Drag(sf::Vector2f(event.mouseMove.x, event.mouseMove.y));
            break;

        default:
            break;
        }
    }
    Redraw();
}

void PolygonalViewer::Select(sf::Vector2f const &point_screen)
{
    geos::geom::Point *point = Reproject(point_screen);

    selection_.clear();
    BOOST_FOREACH (kenv::Object::Ptr object, env_->getObjects()) {
        kenv::PolygonalObject::Ptr polygonal_object = boost::dynamic_pointer_cast<kenv::PolygonalObject>(object);
        boost::shared_ptr<geos::geom::Geometry const> geom = polygonal_object->getGeometry();

        if (geom->intersects(point)) {
            selection_.insert(object);
            break;
        }
    }
}

void PolygonalViewer::Drag(sf::Vector2f const &cursor_curr)
{
    geos::geom::Point *point_curr = Reproject(cursor_curr);
    geos::geom::Point *point_prev = Reproject(cursor_prev_);

    if (dragging_) {
        Eigen::Vector3d delta;
        delta[0] = point_curr->getX() - point_prev->getX();
        delta[1] = point_curr->getY() - point_prev->getY();
        delta[2] = 0;

        BOOST_FOREACH (kenv::Object::Ptr object, selection_) {
            Eigen::Affine3d pose = object->getTransform();
            pose.pretranslate(delta);
            object->setTransform(pose);
        }
    }

    cursor_prev_ = cursor_curr;
    delete point_curr;
    delete point_prev;
}

void PolygonalViewer::Redraw()
{
    window_->clear(background_color_);

    // Draw visible objects.
    BOOST_FOREACH (kenv::Object::Ptr object, env_->getObjects()) {
        kenv::PolygonalObject::Ptr polygonal_object = boost::dynamic_pointer_cast<kenv::PolygonalObject>(object);
        if (!polygonal_object->getVisible()) {
            continue;
        }
        boost::shared_ptr<geos::geom::Geometry const> geom = polygonal_object->getGeometry();

        boost::optional<sf::Color> color;
        if (selection_.count(object) > 0) {
            color.reset(sf::Color(255, 0, 0, 255));
        } else {
            color.reset(toSFMLColor(polygonal_object->getColor()));
        }

        std::vector<sf::Drawable *> drawables;
        FromGeometry(geom.get(), drawables, color);

        BOOST_FOREACH (sf::Drawable *drawable, drawables) {
            window_->draw(*drawable);
            delete drawable;
        }
    }

    // Draw visualization geometry.
    std::vector<ColoredGeometry::Ptr> viz_geom = env_->getVisualizationGeometry();
    BOOST_FOREACH (ColoredGeometry::Ptr custom_geom, viz_geom) {
        std::vector<sf::Drawable *> drawables;
        FromGeometry(custom_geom->geom, drawables, toSFMLColor(custom_geom->color));

        BOOST_FOREACH (sf::Drawable *drawable, drawables) {
            window_->draw(*drawable);
            delete drawable;
        }
    }

    window_->display();
}

void PolygonalViewer::FromGeometry(geos::geom::Geometry const *geom, std::vector<sf::Drawable *> &shapes,
                                   boost::optional<sf::Color> color)
{
    BOOST_ASSERT(geom);

    switch (geom->getGeometryTypeId()) {
    case geos::geom::GEOS_POLYGON:
        FromPolygon(dynamic_cast<geos::geom::Polygon const *>(geom), shapes, color);
        break;

    case geos::geom::GEOS_LINESTRING:
        FromLineString(dynamic_cast<geos::geom::LineString const *>(geom), shapes, color);
        break;

    case geos::geom::GEOS_MULTIPOINT:
    case geos::geom::GEOS_MULTIPOLYGON:
    case geos::geom::GEOS_MULTILINESTRING:
    case geos::geom::GEOS_GEOMETRYCOLLECTION:
        FromGeometryCollection(dynamic_cast<geos::geom::GeometryCollection const *>(geom), shapes, color);
        break;

    default:
        throw std::runtime_error(boost::str(
            boost::format("Unable to render geometry of type [%s].")
                % geom->getGeometryType()
        ));
    }
}

void PolygonalViewer::FromPolygon(geos::geom::Polygon const *polygon, std::vector<sf::Drawable *> &shapes,
                                  boost::optional<sf::Color> color)
{
    BOOST_ASSERT(polygon);

    sf::Color const actual_color = (color) ? *color : sf::Color(255, 255, 255, 255);

    geos::geom::CoordinateSequence const *coordinates = polygon->getCoordinates(); 
    sf::ConvexShape *convex_shape = new sf::ConvexShape(coordinates->getSize());
    convex_shape->setFillColor(sf::Color::Transparent);
    convex_shape->setOutlineThickness(1);
    convex_shape->setOutlineColor(actual_color);

    for (size_t j = 0; j < coordinates->getSize(); ++j) {
        geos::geom::Coordinate const &coord = coordinates->getAt(j);
        convex_shape->setPoint(j, Project(coord));
    }

    delete coordinates;
    shapes.push_back(convex_shape);
}

void PolygonalViewer::FromGeometryCollection(geos::geom::GeometryCollection const *multipolygon,
                                             std::vector<sf::Drawable *> &shapes,
                                             boost::optional<sf::Color> color)
{
    BOOST_ASSERT(multipolygon);

    for (size_t i = 0; i < multipolygon->getNumGeometries(); ++i) {
        geos::geom::Geometry const *geom = multipolygon->getGeometryN(i);
        FromGeometry(geom, shapes, color);
    }
}

void PolygonalViewer::FromLineString(geos::geom::LineString const *linestring,
                                     std::vector<sf::Drawable *> &shapes,
                                     boost::optional<sf::Color> color)
{
    BOOST_ASSERT(linestring);

    sf::Color const actual_color = (color) ? *color : sf::Color(255, 255, 255, 255);

    geos::geom::CoordinateSequence const *coordinates = linestring->getCoordinatesRO();
    sf::VertexArray *linestrip = new sf::VertexArray(sf::LinesStrip, coordinates->getSize());

    for (size_t i = 0; i < coordinates->getSize(); ++i) {
        geos::geom::Coordinate const &coord = coordinates->getAt(i);
        (*linestrip)[i] = sf::Vertex(Project(coord), actual_color);
    }

    shapes.push_back(linestrip);
}

sf::Vector2f PolygonalViewer::Project(geos::geom::Coordinate const &coord) const
{
    sf::Vector2u const window_size = window_->getSize();
    double const px = (coord.x - origin_[0]) * scale_ + window_size.x / 2;
    double const py = (coord.y - origin_[1]) * scale_ + window_size.y / 2;
    return sf::Vector2f(px, py);
}

geos::geom::Point *PolygonalViewer::Reproject(sf::Vector2f const &p) const
{
    geos::geom::GeometryFactory const *geom_factory = geos::geom::GeometryFactory::getDefaultInstance();

    sf::Vector2u const window_size = window_->getSize();
    geos::geom::Coordinate coord;
    coord.x = (p.x - window_size.x / 2) / scale_ + origin_[0];
    coord.y = (p.y - window_size.y / 2) / scale_ + origin_[1];
    return geom_factory->createPoint(coord);
}

sf::Color PolygonalViewer::toSFMLColor(Eigen::Vector4d const &color) const
{
    sf::Color sfml_color;
    sfml_color.r = static_cast<uint8_t>(255 * color[0]);
    sfml_color.g = static_cast<uint8_t>(255 * color[1]);
    sfml_color.b = static_cast<uint8_t>(255 * color[2]);
    sfml_color.a = static_cast<uint8_t>(255 * color[3]);
    return sfml_color;
}

}
