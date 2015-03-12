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
    void flocking();
    void socialForces();

    float getPerceiveRad()const;
    float getPerceiveAng()const;

    void clearNeighbours();
    void clearBoundary();
    void addNeighbour(boost::shared_ptr<Agent> _neighbour);
    void addBoundary(Boundary *_boundary);

private:

    System *m_system;
    Agent *m_agent;
    Avoidance m_avoidanceType;

    std::vector< boost::shared_ptr<Agent> > m_neighbours;
    std::vector<Boundary*> m_Boundaries;


    ngl::Vec3 m_desVel;
    //ngl::Vec3 m_newVel;
    ngl::Vec3 m_goal;
    ngl::Vec3 m_nextGoal;
    std::vector<ngl::Vec3> m_wayPoints;

    float m_perceiveRad;
    float m_perceiveAng;

};



#endif //_BRAIN__H_
