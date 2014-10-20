#pragma once
#include <list>
#include <iostream>
#include <thread>
#include <mutex>
#include <Windows.h>

#include "searchspace.h"

namespace pathfinder {

struct Resource {
    SearchSpace sSpace;
    const int mapSize;
    const unsigned char* const map;
    
    Resource(const SearchSpace& nSearchSpace, const int nMapSize, const unsigned char* const pMap) : sSpace(nSearchSpace), mapSize(nMapSize), map(pMap) {};
    ~Resource() {
        delete[] map;
    }
};

class ObjectPool {
public:
    ObjectPool();
    static ObjectPool* getInstance();
    void addResource(const int nMapWidth, const int nMapHeight, const SearchSpace& nSearchSpace, const unsigned char* const pMap);
    Resource* getResource(const int nMapWidth, const int nMapHeight, const Vec2& nStart, const Vec2& nTarget, const unsigned char* const pMap);
    ~ObjectPool();
private:    
    std::list<Resource*> m_resources;
    std::mutex m_mutex;
    
    std::list<Resource*>::iterator findResource(const int nMapWidth, const int nMapHeight, const Vec2& nStart, const Vec2& nTarget, const unsigned char* const pMap);

};

}  // namespace pathfinder
