#pragma once

namespace pathfinder {

struct Vec2 {
    int x;
    int y;
    Vec2(int x, int y);
    Vec2();
    Vec2(const Vec2&);
    bool equal(const Vec2& other) const;
};

}  // namespace pathfinder
