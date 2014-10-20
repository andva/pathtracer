#include "objectpool.h"

namespace pathfinder {

ObjectPool::ObjectPool() {

}

ObjectPool::~ObjectPool() {
    while (!m_resources.empty()) delete m_resources.front(), m_resources.pop_front();
}



void ObjectPool::addResource(const int nMapWidth, const int nMapHeight, const SearchSpace& nSearchSpace, const unsigned char* const pMap) {
    const int MAX_RESOURCES = 2;
    std::lock_guard<std::mutex> lock(m_mutex);
    std::list<Resource*>::iterator it = findResource(nMapWidth, nMapHeight, nSearchSpace.getStart(), nSearchSpace.getTarget(), pMap);
    if (it != m_resources.end()) {

    }
    else {
        if (m_resources.size() == MAX_RESOURCES) {
            m_resources.pop_front();
        }
        m_resources.push_back(new Resource(nSearchSpace, nMapWidth * nMapHeight, pMap));
    }
}

Resource* ObjectPool::getResource(const int nMapWidth, const int nMapHeight, const Vec2& nStart, const Vec2& nTarget, const unsigned char* const pMap) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_resources.empty()) {
        std::cout << "No resources!";
    }
    else {
        std::list<Resource*>::iterator it = findResource(nMapWidth, nMapHeight, nStart, nTarget, pMap);
        if (it != m_resources.end()) {
            return *it;
        }
    }
    return nullptr;
}

std::list<Resource*>::iterator ObjectPool::findResource(const int nMapWidth, const int nMapHeight, const Vec2& nStart, const Vec2& nTarget, const unsigned char* const pMap) {
    std::list<Resource*>::iterator it;
    for (it = m_resources.begin(); it != m_resources.end(); ++it) {
        const Vec2& ssStart = (*it)->sSpace.getStart();
        const Vec2& ssTarget = (*it)->sSpace.getTarget();

        if (ssStart.x == nStart.x && ssStart.y == nStart.y && ssTarget.x == nTarget.x && ssTarget.y == nTarget.y && nMapWidth * nMapHeight == (*it)->mapSize) {
            bool sameMap = true;
            for (int i = 0; i < nMapWidth * nMapHeight; ++i) {
                if (pMap[i] != (*it)->map[i]) {
                    sameMap = false;
                    break;
                }
            }
            if (sameMap)
                return it;
            break;
        }
    }
    return it;
}

} //  namespace pathfinder
