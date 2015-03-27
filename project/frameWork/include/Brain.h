#ifndef _BRAIN__H_
#define _BRAIN__H_

#include <vector>
#include<boost/shared_ptr.hpp>
#include <ngl/Vec3.h>

class System;
class Agent;
class Boundary;

enum Avoidance {FLOCKING,RVO,SOCIAL};

class Brain
{
public:

    Brain(Agent *_agent, System *_system);
    ~Brain();

    void update();
    void setSystem(System *_system);
    void setAvoidanceType(const Avoidance &_avoidance);
    void setGoal(const ngl::Vec3 &_goal);
    void mapRoute();
    void findNextGoal();
    void findNeighbours();
    void findBoundaries();

    void rvo();
    float cross2D(const ngl::Vec3 &_v1,const ngl::Vec3 &_v2);

    void flocking();
    ngl::Vec3 flockSeparation();
    ngl::Vec3 flockCohesion();
    ngl::Vec3 flockAlignment();
    ngl::Vec3 flockGoal();

    void socialForces();


    float getPerceiveRad()const;
    float getPerceiveAng()const;
    float getDesSpeed()const;

    void clearNeighbours();
    void clearBoundary();
    void addNeighbour(Agent* _neighbour);
    void addBoundary(Boundary *_boundary);

    void printInfo();

private:

    System *m_system;
    Agent *m_agent;
    Avoidance m_avoidanceType;

    std::vector< Agent* > m_neighbours;
    std::vector<Boundary*> m_Boundaries;


    ngl::Vec3 m_desVel;
    float m_desSpeed;
    float m_maxVel;
    float m_maxAcc;
    //ngl::Vec3 m_newVel;
    ngl::Vec3 m_goal;
    ngl::Vec3 m_nextGoal;
    std::vector<ngl::Vec3> m_wayPoints;

    float m_perceiveRad;
    float m_perceiveAng;

    //flocking stuff
    float m_goalWeight;
    float m_alignmentWeight;
    float m_cohesionWeight;
    float m_separationWeight;


};



#endif //_BRAIN__H_
