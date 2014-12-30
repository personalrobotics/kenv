#ifndef BOX2DVISUALIZER_H_
#define BOX2DVISUALIZER_H_
#include <Box2D/Common/b2Draw.h>
#include <SFML/Graphics.hpp>

namespace box2d_kenv {

class Box2DVisualizer : public b2Draw {
public:
    Box2DVisualizer(sf::RenderWindow *window, double scale,
                    double center_x, double center_y);

	virtual void DrawPolygon(b2Vec2 const *vertices, int32 vertexCount,
                             b2Color const &color);

	virtual void DrawSolidPolygon(b2Vec2 const *vertices, int32 vertexCount,
                                  b2Color const &color);

	virtual void DrawCircle(b2Vec2 const &center, float32 radius,
                            b2Color const &color);

	virtual void DrawSolidCircle(b2Vec2 const &center, float32 radius,
                                 b2Vec2 const &axis, b2Color const &color);

	virtual void DrawSegment(b2Vec2 const &p1, const b2Vec2& p2,
                             b2Color const &color);

	virtual void DrawTransform(b2Transform const &xf);

private:
    sf::RenderWindow *window_;
    double center_x_;
    double center_y_;
    double scale_;
    double axis_length_;

    sf::Color b2ToColor(b2Color const &color) const;
    sf::Vector2f b2ToVector(b2Vec2 const &p) const;
    sf::Vertex b2ToVertex(b2Vec2 const &p) const;
};

}

#endif
