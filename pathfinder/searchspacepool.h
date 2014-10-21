#pragma once
#include <list>
#include <iostream>
#include <thread>
#include <mutex>
#include <Windows.h>
#include <memory>

#include "searchspace.h"
#include "resource.h"

namespace pathfinder {

class SearchSpacePool {
public:
    
    void addResource(const int mapWidth, const int mapHeight, const SearchSpace& nSearchSpace, const unsigned char* const map);

    // Returns copy to make sure that other thread does not delete a reference.
    // If not found, returns new SearchSpace object.
    SearchSpace getResource(const int mapWidth, const int mapHeight, const int maxSteps, const Vec2& nStart, const Vec2& nTarget, const unsigned char* const map);
    
    static SearchSpacePool& singleton();
protected:
    ~SearchSpacePool();

private:
    SearchSpacePool();
    SearchSpacePool(const SearchSpacePool& other) = delete;
    SearchSpacePool& operator=(const SearchSpacePool&) = delete;
    
    static SearchSpacePool* s_objectPool;

    static std::once_flag s_singleton_created_flag;

    
    std::list<Resource*> m_resources;
    std::mutex m_mutex;
    
    std::list<Resource*>::iterator findResource(const int mapWidth, const int mapHeight, const Vec2& nStart, const Vec2& nTarget, const unsigned char* const map);

};

}  // namespace pathfinder
