#include "Brain.h"

#include <iostream>
#include <math.h>
#include <boost/foreach.hpp>

#include "Agent.h"

Brain::Brain(Agent *_agent, System *_system)
{
    // ctor that should be used
    m_agent = _agent;
    m_system = _system;

    m_perceiveRad = 1.0f;
    m_perceiveAng = 360.0f;
}

Brain::~Brain()
{

}

void Brain::update()
{
    static float z = 0.0f;
    static float x = 0.0f;

    findNextGoal();
    //m_goal.m_z = sin(z+=0.0001)*10-5;
    //m_goal.m_x = cos(x+=0.0001)*10 -5;


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

    std::cout<<"num neighbours "<<m_neighbours.size()<<"\n";

    //clearNeighbours();
//    clearBoundary();
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
    //m_goal = ?
}

void Brain::findNeighbours()
{

}

void Brain::findBoundaries()
{

}

//===============RVO==================
void Brain::rvo()
{
    //std::cout<<"RVO avoidance in use\n"<<std::endl;
}

//=============flocking===============
void Brain::flocking()
{
    float goalWeight = 0.1f;
    //---------------goal rule----------------------
    ngl::Vec3 goal = m_goal - m_agent->getOrigState().m_pos;
    if(goal != ngl::Vec3(0.0f,0.0f,0.0f))
    {
        goal.normalize();
        goal *= goalWeight;
    }
    else
    {
        m_agent->setForce(goal);
        return;
    }
    //----------------------------------------------

    //-------------check for neigbours--------------
    if(m_neighbours.size() == 0)
    {
        m_agent->setForce(goal);
        return;
    }
    //----------------------------------------------

    float alignmentWeight = 0.5f;
    float cohesionWeight = 0.8f;
    float separationWeight = 0.2f;

    //------------separation rule-------------------
    ngl::Vec3 separation;
    BOOST_FOREACH(boost::shared_ptr<Agent> n, m_neighbours)
    {
        float dist2 = (m_agent->getOrigState().m_pos - n->getOrigState().m_pos).lengthSquared() -
                     pow((m_agent->getOrigState().m_rad + n->getOrigState().m_rad),2);
        if(dist2 < pow(2*m_perceiveRad,1))
        {
            separation -= (n->getOrigState().m_pos - m_agent->getOrigState().m_pos);
            separation *= (m_perceiveRad/dist2);
        }
    }
    if(separation != ngl::Vec3(0.0f,0.0f,0.0f))
    {
        separation.normalize();
        //separation /= m_neighbours.size();
        goal *= separationWeight;
    }

    //----------------------------------------------

    //------------Alignment rule--------------------
    ngl::Vec3 alignment;
    BOOST_FOREACH(boost::shared_ptr<Agent> n, m_neighbours)
    {
        alignment += n->getOrigState().m_vel;
    }
    if(alignment != ngl::Vec3(0.0f,0.0f,0.0f))
    {
        alignment.normalize();
        //alignment /= m_neighbours.size();
        alignment*= alignmentWeight;
    }
    //----------------------------------------------

    //----------Cohesion rule----------------------
    ngl::Vec3 cohesion;
    BOOST_FOREACH(boost::shared_ptr<Agent> n, m_neighbours)
    {
        cohesion += n->getOrigState().m_pos;
    }
    if(cohesion!= ngl::Vec3(0.0f,0.0f,0.0f))
    {
        cohesion.normalize();
        //cohesion /= m_neighbours.size();
        cohesion *= cohesionWeight;
    }
    //----------------------------------------------

    //-------------Final force----------------------
    ngl::Vec3 finalForce = (separation + alignment + cohesion + goal);
    //finalForce.normalize();
    if(finalForce != ngl::Vec3(0.0f,0.0f,0.0f))
    {
        finalForce.normalize();
    }

    m_agent->setForce(finalForce);
    //m_agent->setVel(0.01*finalForce);
    //----------------------------------------------
}

//----------------Social forces-----------------------
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
    for(unsigned int i=0;i<m_neighbours.size();i++)
    {
        //delete [] m_neighbours[i];
        //m_neighbours.pop_back();
    }
    m_neighbours.clear();
    //m_neighbours.erase(m_neighbours.begin(),m_neighbours.end());
}

void Brain::clearBoundary()
{
    //m_Boundaries.clear();
    //m_Boundaries.erase(m_Boundaries.begin(),m_Boundaries.end());

    for(unsigned int i=0;i<m_Boundaries.size();i++)
    {
        m_Boundaries[i] == NULL;
        m_Boundaries.pop_back();
    }
}

void Brain::addNeighbour( boost::shared_ptr<Agent> _neighbour)
{
    m_neighbours.push_back(_neighbour);
}

void Brain::addBoundary(Boundary *_boundary)
{
    m_Boundaries.push_back(_boundary);
}
