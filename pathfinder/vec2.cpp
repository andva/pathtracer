#include "vec2.h"

namespace pathfinder {

Vec2::Vec2(int x, int y)
    : x(x)
    , y(y)
{ }

Vec2::Vec2()
    : x(0)
    , y(0)
{ }

Vec2::Vec2(const Vec2& other)
    : x(other.x)
    , y(other.y)
{ }

bool Vec2::equal(const Vec2& other) const {
    return (x == other.x && y == other.y);
}

}  // namespace pathfinder
