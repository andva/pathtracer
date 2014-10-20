#include "Resource.h"

#include <cstdio>
#include "searchspace.h"

namespace pathfinder {

Resource::Resource(const SearchSpace& nSearchSpace, const int nMapSize, const unsigned char* const pMap) :
sSpace(nSearchSpace),
mapSize(nMapSize) {
    map = new unsigned char[mapSize];
    memcpy(map, pMap, nMapSize * sizeof(unsigned char));
};

Resource::~Resource() {
    delete[] map;
}

}