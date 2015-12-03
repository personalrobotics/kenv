#include <iostream>
#include <stdint.h>
#include <boost/assert.hpp>
#include "Box2DVisualizer.h"

namespace box2d_kenv {

Box2DVisualizer::Box2DVisualizer(sf::RenderWindow *window, double scale,
                                 double center_x, double center_y)
    : window_(window)
    , scale_(scale)
    , center_x_(center_x)
    , center_y_(center_y)
    , axis_length_(0.1 * scale)
{
    BOOST_ASSERT(window_);
    BOOST_ASSERT(scale_ > 0.);
}

void Box2DVisualizer::DrawPolygon(b2Vec2 const *vertices, int32 vertexCount,
                                  b2Color const &color)
{
    sf::ConvexShape polygon(vertexCount);

    for (int32 ivertex = 0; ivertex < vertexCount; ++ivertex) {
        polygon.setPoint(ivertex, b2ToVector(vertices[ivertex]));
    }

    polygon.setFillColor(sf::Color::Transparent);
    polygon.setOutlineColor(b2ToColor(color));
    polygon.setOutlineThickness(1);

    window_->draw(polygon);
}

void Box2DVisualizer::DrawSolidPolygon(b2Vec2 const *vertices, int32 vertexCount,
                                       b2Color const &color)
{
    sf::ConvexShape polygon(vertexCount);

    for (int32 ivertex = 0; ivertex < vertexCount; ++ivertex) {
        polygon.setPoint(ivertex, b2ToVector(vertices[ivertex]));
    }

    polygon.setFillColor(b2ToColor(color));
    polygon.setOutlineColor(sf::Color::Transparent);

    window_->draw(polygon);
}

void Box2DVisualizer::DrawCircle(b2Vec2 const &center, float32 radius,
                                 b2Color const &color)
{
    sf::CircleShape circle(scale_ * radius);
    circle.setPosition(scale_ * center.x + center_x_,
                       scale_ * center.y + center_y_);
    circle.setFillColor(sf::Color::Transparent);
    circle.setOutlineColor(b2ToColor(color));
    circle.setOutlineThickness(1);

    window_->draw(circle);
}

void Box2DVisualizer::DrawSolidCircle(b2Vec2 const &center, float32 radius,
                                      b2Vec2 const &axis, b2Color const &color)
{
    sf::CircleShape circle(scale_ * radius);
    circle.setPosition(scale_ * center.x + center_x_,
                       scale_ * center.y + center_y_);
    circle.setFillColor(b2ToColor(color));
    circle.setOutlineColor(sf::Color::Transparent);

    window_->draw(circle);
}

void Box2DVisualizer::DrawSegment(b2Vec2 const &p1, const b2Vec2& p2,
                                  b2Color const &color)
{
    sf::Vertex line[] = { b2ToVertex(p1), b2ToVertex(p2) };
    line[0].color = b2ToColor(color);
    line[1].color = b2ToColor(color);

    window_->draw(line, 2, sf::Lines);
}

void Box2DVisualizer::DrawTransform(b2Transform const &xf)
{
    static b2Color const color_x(1., 0., 0.);
    static b2Color const color_y(0., 1., 0.);

    DrawSegment(xf.p, xf.p + axis_length_ * xf.q.GetXAxis(), color_x);
    DrawSegment(xf.p, xf.p + axis_length_ * xf.q.GetYAxis(), color_y);
}

sf::Color Box2DVisualizer::b2ToColor(b2Color const &color) const
{
    BOOST_ASSERT(0. <= color.r && color.r <= 1.);
    BOOST_ASSERT(0. <= color.g && color.g <= 1.);
    BOOST_ASSERT(0. <= color.b && color.b <= 1.);

    return sf::Color(static_cast<uint8_t>(255 * color.r),
                     static_cast<uint8_t>(255 * color.g),
                     static_cast<uint8_t>(255 * color.b));
}

sf::Vector2f Box2DVisualizer::b2ToVector(b2Vec2 const &p) const
{
    return sf::Vector2f(scale_ * p.x + center_x_,
                        scale_ * p.y + center_y_);
}

sf::Vertex Box2DVisualizer::b2ToVertex(b2Vec2 const &p) const
{
    return sf::Vertex(b2ToVector(p));
}

}
