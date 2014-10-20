#include "Resource.h"

#include <cstdio>
#include "searchspace.h"

namespace pathfinder {

Resource::Resource(const SearchSpace& searchSpace, const int nMapSize, const unsigned char* const mapOriginal)
    : searchSpace(searchSpace)
    , mapSize(nMapSize)
{
    map = new unsigned char[mapSize];
    memcpy(map, mapOriginal, nMapSize * sizeof(unsigned char));
};

Resource::~Resource() {
    delete[] map;
}

}