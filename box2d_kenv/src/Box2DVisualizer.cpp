#include <iostream>
#include "util/Box2DVisualizer.h"

namespace manifold_pf3 {
namespace util {

Box2DVisualizer::Box2DVisualizer(unsigned int width, unsigned int height,
                                 double offset_x, double offset_y,
                                 double scale, std::string const &name)
    : window_(sf::VideoMode(width, height), name)
    , color_fill_(190, 174, 212)
    , color_edge_(253, 192, 134)
    , offset_x_(offset_x)
    , offset_y_(offset_y)
    , scale_(scale)
{
}

sf::RenderWindow &Box2DVisualizer::window()
{
    return window_;
}

void Box2DVisualizer::Step()
{
    sf::Event event;
    while (window_.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
            window_.close();
        }
    }
}

void Box2DVisualizer::Spin()
{
    while (!IsDone()) {
        Step();

        window_.display();
    }
}

bool Box2DVisualizer::IsDone() const
{
    return !window_.isOpen();
}

void Box2DVisualizer::DrawPolygon(b2Vec2 const *vertices, int32 vertexCount,
                                  b2Color const &color)
{
    sf::ConvexShape polygon(vertexCount);

    for (int32 ivertex = 0; ivertex < vertexCount; ++ivertex) {
        polygon.setPoint(ivertex, b2ToVector(vertices[ivertex]));
    }

    polygon.setFillColor(sf::Color::Transparent);
    polygon.setOutlineColor(color_edge_);
    polygon.setOutlineThickness(1);

    window_.draw(polygon);
}

void Box2DVisualizer::DrawSolidPolygon(b2Vec2 const *vertices, int32 vertexCount,
                                       b2Color const &color)
{
    sf::ConvexShape polygon(vertexCount);

    for (int32 ivertex = 0; ivertex < vertexCount; ++ivertex) {
        polygon.setPoint(ivertex, b2ToVector(vertices[ivertex]));
    }

    polygon.setFillColor(color_fill_);
    polygon.setOutlineColor(sf::Color::Transparent);

    window_.draw(polygon);
}

void Box2DVisualizer::DrawCircle(b2Vec2 const &center, float32 radius,
                                 b2Color const &color)
{
    sf::CircleShape circle(scale_ * radius);
    circle.setPosition(scale_ * center.x - offset_x_,
                       scale_ * center.y - offset_y_);
    circle.setFillColor(sf::Color::Transparent);
    circle.setOutlineColor(color_edge_);
    circle.setOutlineThickness(1);

    window_.draw(circle);
}

void Box2DVisualizer::DrawSolidCircle(b2Vec2 const &center, float32 radius,
                                      b2Vec2 const &axis, b2Color const &color)
{
    sf::CircleShape circle(scale_ * radius);
    circle.setPosition(scale_ * center.x - offset_x_,
                       scale_ * center.y - offset_y_);
    circle.setFillColor(color_fill_);
    circle.setOutlineColor(sf::Color::Transparent);

    window_.draw(circle);
}

void Box2DVisualizer::DrawSegment(b2Vec2 const &p1, const b2Vec2& p2,
                                  b2Color const &color)
{
    sf::Vertex line[] = { b2ToVertex(p1), b2ToVertex(p2) };
    line[0].color = color_edge_;
    line[1].color = color_edge_;

    window_.draw(line, 2, sf::Lines);
}

void Box2DVisualizer::DrawTransform(b2Transform const &xf)
{
    // TODO: Implement this.
}

sf::Vector2f Box2DVisualizer::b2ToVector(b2Vec2 const &p) const
{
    return sf::Vector2f(scale_ * p.x - offset_x_,
                        scale_ * p.y - offset_y_);
}

sf::Vertex Box2DVisualizer::b2ToVertex(b2Vec2 const &p) const
{
    return sf::Vertex(b2ToVector(p));
}


}
}
