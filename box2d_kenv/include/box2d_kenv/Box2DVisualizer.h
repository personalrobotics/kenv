#ifndef BOX2DVISUALIZER_H_
#define BOX2DVISUALIZER_H_
#include <Box2D/Common/b2Draw.h>
#include <SFML/Graphics.hpp>

namespace manifold_pf3 {
namespace util {

class Box2DVisualizer : public b2Draw {
public:
    Box2DVisualizer(unsigned int width, unsigned int height,
                    double offset_x, double offset_y,
                    double scale, std::string const &name);

    void Step();
    void Spin();
    bool IsDone() const;

    sf::RenderWindow &window();

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
    sf::RenderWindow window_;
    sf::Color color_fill_;
    sf::Color color_edge_;
    double offset_x_;
    double offset_y_;
    double scale_;

    sf::Vector2f b2ToVector(b2Vec2 const &p) const;
    sf::Vertex b2ToVertex(b2Vec2 const &p) const;
};

}
}
#endif
