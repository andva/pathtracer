#pragma once

namespace pathfinder {
struct Vec2 {
    int x;
    int y;
    Vec2(int x, int y);
    Vec2();
    Vec2(const Vec2&);
};
}  // namespace pathfinder
