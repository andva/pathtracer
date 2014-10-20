#pragma once

#include "searchspace.h"

namespace pathfinder {


struct Resource {
    SearchSpace searchSpace;
    const int mapSize;
    unsigned char* map;

    Resource(const SearchSpace& searchSpace, const int mapSize, const unsigned char* const map);
    ~Resource();
private:
    Resource(const Resource& other);
};
}  //  namespace pathfinder