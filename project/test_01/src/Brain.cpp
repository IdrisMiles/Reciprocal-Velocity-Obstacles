#include "Brain.h"

#include <iostream>
#include <math.h>

#include "Agent.h"

Brain::Brain(Agent *_agent, System *_system)
{
    // ctor that should be used
    m_agent = _agent;
    m_system = _system;

    // randomly place Agent
}

Brain::~Brain()
{

}

void Brain::update()
{
    findNextGoal();

    switch (m_avoidanceType)
    {
        case FLOCKING:
            flocking();break;
        case RVO:
            rvo();break;
        case SOCIAL:
            socialForces();break;
        default:
            break;
    }

    //std::cout<<"num neighbours "<<m_neighbours.size()<<"\n";

    clearNeighbours();
    clearBoundary();
}

void Brain::setSystem(System *_system)
{
    m_system = _system;
}

void Brain::setAvoidanceType(const Avoidance &_avoidance)
{
    m_avoidanceType = _avoidance;
}

void Brain::setGoal(const ngl::Vec3 &_goal)
{
    m_goal = _goal;
}

void Brain::mapRoute()
{

}

void Brain::findNextGoal()
{

}

void Brain::findNeighbours()
{

}

void Brain::findBoundaries()
{

}

void Brain::rvo()
{
    //std::cout<<"RVO avoidance in use\n"<<std::endl;
}

void Brain::flocking()
{
    static float f = 0.0f;
    //std::cout<<"flocking avoidance in use\n"<<std::endl;
    ngl::Vec3 separation;
    //separation.normalize();
    ngl::Vec3 alignment;
    //alignment.normalize();
    ngl::Vec3 cohesion;
    //cohesion.normalize();

    //ngl::Vec3 finalForce = 0.01 * (separation + alignment + cohesion);
    ngl::Vec3 finalForce = ngl::Vec3(f,0.0f,0.0f);
    f=0.1;

    m_agent->setForce(finalForce);
}

void Brain::socialForces()
{
    //std::cout<<"social forces avoidance in use\n"<<std::endl;
}

float Brain::getPerceiveRad()const
{
    return m_perceiveRad;
}

float Brain::getPerceiveAng()const
{
    return m_perceiveAng;
}


void Brain::clearNeighbours()
{
    m_neighbours.clear();
}

void Brain::clearBoundary()
{
    m_Boundaries.clear();
}

void Brain::addNeighbour(Agent *_neighbour)
{
    m_neighbours.push_back(_neighbour);
}

void Brain::addBoundary(Boundary *_boundary)
{
    m_Boundaries.push_back(_boundary);
}
