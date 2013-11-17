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
    boost::shared_ptr<geos::geom::Point> point_curr(Reproject(cursor_curr));
    boost::shared_ptr<geos::geom::Point> point_prev(Reproject(cursor_prev_));

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
}

void PolygonalViewer::Redraw()
{
    std::vector<sf::Drawable *> drawables;
    window_->clear(background_color_);

    // Texture patches.
    std::vector<TexturePatch::Ptr> texture_patches = env_->getTexturePatches();
    BOOST_FOREACH (TexturePatch::Ptr texture_patch, texture_patches) {
        FromTexturePatch(*texture_patch, drawables);
    }
 
    // Visible objects.
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
        FromGeometry(geom.get(), drawables, color);
    }

    // Visualization geometry.
    std::vector<ColoredGeometry::Ptr> viz_geom = env_->getVisualizationGeometry();
    BOOST_FOREACH (ColoredGeometry::Ptr custom_geom, viz_geom) {
        FromGeometry(custom_geom->geom, drawables, toSFMLColor(custom_geom->color));
    }

    // Redraw the window.
    BOOST_FOREACH (sf::Drawable *drawable, drawables) {
        window_->draw(*drawable);
        delete drawable;
    }
    window_->display();

    // Clear the texture buffer.
    BOOST_FOREACH (sf::Texture *texture, texture_buffer_) {
        delete texture;
    }
    texture_buffer_.clear();
}

void PolygonalViewer::FromTexturePatch(TexturePatch const &patch, std::vector<sf::Drawable *> &shapes)
{
    sf::Image image;
    size_t const *shape = patch.texture.shape();
    image.create(shape[0], shape[1], sf::Color::Transparent);

    // Grayscale.
    if (shape[2] == 1) {
        for (size_t x = 0; x < shape[0]; ++x)
        for (size_t y = 0; y < shape[1]; ++y) {
            sf::Color color;
            color.r = static_cast<uint8_t>(255 * patch.texture[x][y][0]);
            color.g = static_cast<uint8_t>(255 * patch.texture[x][y][0]);
            color.b = static_cast<uint8_t>(255 * patch.texture[x][y][0]);
            color.a = 255;
            image.setPixel(x, y, color);
        }
    // RGB.
    } else if (shape[2] == 3) {
        for (size_t x = 0; x < shape[0]; ++x)
        for (size_t y = 0; y < shape[1]; ++y) {
            sf::Color color;
            color.r = static_cast<uint8_t>(255 * patch.texture[x][y][0]);
            color.g = static_cast<uint8_t>(255 * patch.texture[x][y][1]);
            color.b = static_cast<uint8_t>(255 * patch.texture[x][y][2]);
            color.a = 255;
            image.setPixel(x, y, color);
        }
    // RGBA.
    } else if (shape[2] == 4) {
        for (size_t x = 0; x < shape[0]; ++x)
        for (size_t y = 0; y < shape[1]; ++y) {
            sf::Color color;
            color.r = static_cast<uint8_t>(255 * patch.texture[x][y][0]);
            color.g = static_cast<uint8_t>(255 * patch.texture[x][y][1]);
            color.b = static_cast<uint8_t>(255 * patch.texture[x][y][2]);
            color.a = static_cast<uint8_t>(255 * patch.texture[x][y][3]);
            image.setPixel(x, y, color);
        }
    } else {
        throw std::runtime_error("Texture must have 1, 3, or 4 channels.");
    }

    sf::Texture *image_texture = new sf::Texture;
    texture_buffer_.push_back(image_texture);
    image_texture->loadFromImage(image);
    image_texture->setSmooth(false);
    image_texture->setRepeated(false);

    Eigen::Affine2d const &pose = patch.origin;
    geos::geom::Coordinate const position(pose.translation()[0], pose.translation()[1]);
    double const orientation_rad = std::atan2(pose.linear()(1, 0), pose.linear()(0, 0));
    double const orientation_deg = orientation_rad * 180 / M_PI;

    sf::Sprite *sprite = new sf::Sprite;
    sprite->setTexture(*image_texture);
    sprite->setOrigin(sf::Vector2f(shape[0] / 2, shape[1] / 2));
    sprite->setPosition(Project(position));
    sprite->setRotation(orientation_deg);
    sprite->setScale(sf::Vector2f(scale_ * patch.width / shape[0], scale_ * patch.height / shape[1]));
    shapes.push_back(sprite);
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

    case geos::geom::GEOS_POINT:
        FromPoint(dynamic_cast<geos::geom::Point const *>(geom), shapes, color);
        break;

    case geos::geom::GEOS_MULTIPOINT:
        // We can render the points as a batch more efficiently than individually.
        FromMultiPoint(dynamic_cast<geos::geom::MultiPoint const *>(geom), shapes, color);
        break;

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

    boost::shared_ptr<geos::geom::CoordinateSequence> coordinates(
        polygon->getCoordinates()
    );
    sf::ConvexShape *convex_shape = new sf::ConvexShape(coordinates->getSize());
    convex_shape->setFillColor(sf::Color::Transparent);
    convex_shape->setOutlineThickness(1);
    convex_shape->setOutlineColor(actual_color);

    for (size_t j = 0; j < coordinates->getSize(); ++j) {
        geos::geom::Coordinate const &coord = coordinates->getAt(j);
        convex_shape->setPoint(j, Project(coord));
    }
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

void PolygonalViewer::FromPoint(geos::geom::Point const *point,
                                std::vector<sf::Drawable *> &shapes,
                                boost::optional<sf::Color> color)
{
    BOOST_ASSERT(point);
    geos::geom::Coordinate const *coord = point->getCoordinate();

    sf::Color const actual_color = (color) ? *color : sf::Color(255, 255, 255, 255);
    sf::VertexArray *sf_point = new sf::VertexArray(sf::Points, 1);
    (*sf_point)[0] = sf::Vertex(Project(*coord), actual_color);
    shapes.push_back(sf_point);
}

void PolygonalViewer::FromMultiPoint(geos::geom::MultiPoint const *multipoint,
                                     std::vector<sf::Drawable *> &shapes,
                                     boost::optional<sf::Color> color)
{
    BOOST_ASSERT(multipoint);

    sf::Color const actual_color = (color) ? *color : sf::Color(255, 255, 255, 255);

    boost::shared_ptr<geos::geom::CoordinateSequence> coordinates = boost::shared_ptr<geos::geom::CoordinateSequence>(multipoint->getCoordinates());
    sf::VertexArray *sf_multipoint = new sf::VertexArray(sf::Points, coordinates->getSize());

    for (size_t i = 0; i < coordinates->getSize(); ++i) {
        geos::geom::Coordinate const &coord = coordinates->getAt(i);
        (*sf_multipoint)[i] = sf::Vertex(Project(coord), actual_color);
    }

    shapes.push_back(sf_multipoint);
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
