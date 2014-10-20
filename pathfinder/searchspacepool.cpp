#include "searchspacepool.h"

namespace pathfinder {

SearchSpacePool::SearchSpacePool() {

}

SearchSpacePool::~SearchSpacePool() {
    while (!m_resources.empty()) delete m_resources.front(), m_resources.pop_front();
}

void SearchSpacePool::addResource(const int mapWidth, const int mapHeight, const SearchSpace& nSearchSpace, const unsigned char* const map) {
    const int MAX_RESOURCES = 2;
    std::lock_guard<std::mutex> lock(m_mutex);
    std::list<Resource*>::iterator it = findResource(mapWidth, mapHeight, nSearchSpace.getStart(), nSearchSpace.getTarget(), map);
    if (it != m_resources.end()) {

    }
    else {
        if (m_resources.size() == MAX_RESOURCES) {
            m_resources.pop_front();
        }
        m_resources.push_back(new Resource(nSearchSpace, mapWidth * mapHeight, map));
    }
}

SearchSpace SearchSpacePool::getResource(const int mapWidth, const int mapHeight, const int maxSteps, const Vec2& nStart, const Vec2& nTarget, const unsigned char* const map) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_resources.empty()) {
        std::list<Resource*>::iterator it = findResource(mapWidth, mapHeight, nStart, nTarget, map);
        if (it != m_resources.end()) {
            (*it)->searchSpace.updateForNewMaxSteps(maxSteps);
            SearchSpace sp = (*it)->searchSpace;
            m_resources.remove(*it);
            return sp;
        }
    }
    return SearchSpace(mapWidth, mapHeight, maxSteps, nStart, nTarget);
}

std::list<Resource*>::iterator SearchSpacePool::findResource(const int mapWidth, const int mapHeight, const Vec2& nStart, const Vec2& nTarget, const unsigned char* const map) {
    std::list<Resource*>::iterator it;
    for (it = m_resources.begin(); it != m_resources.end(); ++it) {
        const Vec2& ssStart = (*it)->searchSpace.getStart();
        const Vec2& ssTarget = (*it)->searchSpace.getTarget();

        if (ssStart.equal(nStart) && ssTarget.equal(nTarget) && mapWidth * mapHeight == (*it)->mapSize) {
            bool sameMap = true;
            for (int i = 0; i < mapWidth * mapHeight; ++i) {
                if (map[i] != (*it)->map[i]) {
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
