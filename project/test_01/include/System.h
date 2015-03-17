#ifndef _SYSTEM__H_
#define _SYSTEM__H_

#include <vector>
#include <ngl/Camera.h>
#include <ngl/Mat4.h>
#include "Agent.h"
#include "agentOctree.h"
#include "Boundary.h"

enum SpatialDivision {BRUTE,OCTREE};
class AgentOctree;
class System
{
public:
    System();
    ~System();

    void update();
    void addAgent(const Avoidance &_avoidType);
    void addNeighbours();
    void addBoundaries();
    void clearNeighbours();
    void clearBoundaries();

    void setSpatialDivision(const SpatialDivision &_type);
    void setGloablGoal(const ngl::Vec3 &_goal);

    void draw();
    void setUpDraw(const ngl::Camera &_cam, const ngl::Mat4 &_tx);
    ngl::Camera getCam()const;
    ngl::Mat4 getGlobalTX()const;



private:

    std::vector< boost::shared_ptr<Agent> > m_agents;
    std::vector<Boundary*> m_Boundaries;

    SpatialDivision m_spatialDivision;
    AgentOctree *m_octree;

    ngl::Vec3 m_groundPlane[4];
    ngl::Camera m_cam;
    ngl::Mat4 m_globalTX;

    int numAgents;
    int numBoundaries;

};


#endif //_SYSTEM__H_
