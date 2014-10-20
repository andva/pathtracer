#pragma once

#include "searchspace.h"

namespace pathfinder {


struct Resource {
    SearchSpace sSpace;
    const int mapSize;
    unsigned char* map;

    Resource(const SearchSpace& nSearchSpace, const int nMapSize, const unsigned char* const pMap);
    ~Resource();
private:
    Resource(const Resource& other);
};
}  //  namespace pathfinder