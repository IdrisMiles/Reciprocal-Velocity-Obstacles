#include "Brain.h"

#include <iostream>
#include <math.h>
#include <boost/foreach.hpp>
#include <ngl/NGLStream.h>

#include "Agent.h"

Brain::Brain(Agent *_agent, System *_system)
{
    // ctor that should be used
    m_agent = _agent;
    m_system = _system;

    m_perceiveRad = 1.0f;
    m_perceiveAng = 360.0f;
    m_desSpeed = 0.1;
    m_maxVel = 0.1;
    m_maxAcc = 0.1;

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

    // collision response goes here

    //std::cout<<"num neighbours "<<m_neighbours.size()<<"\n";

}

void Brain::setSystem(System *_system)
{
    m_system = _system;
}

void Brain::setAvoidanceType(const Avoidance &_avoidance)
{
    m_avoidanceType = _avoidance;

    switch (m_avoidanceType)
    {
        case FLOCKING:
            // setting up flocking attibutes
            m_goalWeight = 0.1f;
            m_alignmentWeight = 0.1f;
            m_cohesionWeight = 0.1f;
            m_separationWeight = 0.9f;
            break;
        case RVO:
            break;
        case SOCIAL:
            break;
        default:
            break;
    }

}

void Brain::setGoal(const ngl::Vec3 &_goal)
{
    m_goal = _goal;

    m_desVel = m_goal - m_agent->getOrigState().m_pos;
    m_desVel.normalize();
    m_desVel *= m_desSpeed;
}

void Brain::mapRoute()
{

}

void Brain::findNextGoal()
{
    //m_goal = ?

    m_desVel = m_goal - m_agent->getOrigState().m_pos;
    m_desVel.normalize();
    m_desVel *= m_desSpeed;
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
    // Ray intersection test for current agent with circle around neighbour agent
    // with radius aRad + bRad.

    float time;
    time = 1.0f;
    //std::cout<<"RVO avoidance in use\n"<<std::endl;
    BOOST_FOREACH(Agent *n, m_neighbours)
    {
        float t,u;
        //t = (q − p) × s / (r × s)
        //u = (q − p) × r / (r × s)
        // replace with desired direction
        float dirCross = cross2D(m_agent->getOrigState().m_vel,n->getOrigState().m_vel);
        if(dirCross != 0)
        {
            ngl::Vec3 dist = n->getOrigState().m_pos - m_agent->getOrigState().m_pos;
            float tCross = cross2D(dist,n->getOrigState().m_vel);
            float uCross = cross2D(dist,m_agent->getOrigState().m_vel);

            t = tCross / dirCross;
            u = uCross / dirCross;

            if(t == u)
            {
                if(t>=0 && t<=time)
                {
                    // Agent is going to collide with neighbour in t amount of time
                }
            }
        }
    }

}

float Brain::cross2D(const ngl::Vec3 &_v1, const ngl::Vec3 &_v2)
{
    return (_v1.m_x * _v2.m_z) - (_v1.m_z * _v2.m_x);
}

//=============flocking===============
void Brain::flocking()
{
    //---------------goal rule----------------------
    ngl::Vec3 goal = m_goal - m_agent->getOrigState().m_pos;
    if(goal != ngl::Vec3(0.0f,0.0f,0.0f))
    {
        goal.normalize();
        goal *= m_goalWeight;
    }
    else
    {
        m_agent->setForce(goal);
        return;
    }
    //----------------------------------------------

    //-------------check for neigbours--------------
    if(m_neighbours.size() <1)
    {
        // no neightbours, only goal force will apply
        goal.normalize();
        m_agent->setForce(goal);
        return;
    }

    //------------separation rule-------------------
    ngl::Vec3 separation = flockSeparation();

    //------------Alignment rule--------------------
    ngl::Vec3 alignment = flockAlignment();

    //----------Cohesion rule----------------------
    ngl::Vec3 cohesion = ngl::Vec3(0,0,0);//flockCohesion();

    //-------------Final force----------------------
    ngl::Vec3 finalForce = (separation + alignment + cohesion + goal);
    if(finalForce != ngl::Vec3(0.0f,0.0f,0.0f))
    {
        finalForce.normalize();
    }

    m_agent->setForce(0.01*finalForce);
    //m_agent->setVel(0.01*finalForce);
    //----------------------------------------------
}

ngl::Vec3 Brain::flockSeparation()
{
    ngl::Vec3 separation;
    BOOST_FOREACH(Agent* n, m_neighbours)
    {
        ngl::Vec3 tmpSeparation;
        // separation from neighbouring boids
        ngl::Vec3 distV = m_agent->getOrigState().m_pos - n->getOrigState().m_pos;
        float distF = (m_agent->getOrigState().m_pos - n->getOrigState().m_pos).length() -
                     (m_agent->getOrigState().m_rad + n->getOrigState().m_rad);

        if(distF < m_perceiveRad)
        {
            tmpSeparation -= (n->getOrigState().m_pos - m_agent->getOrigState().m_pos);
            if(distF!=0) {tmpSeparation *= (m_perceiveRad/distF);}
            else         {tmpSeparation *= 1000;}

            if(distF <= m_agent->getOrigState().m_rad)
            {
                // TOO CLOSE
                ngl::Vec3 tmpVel = m_agent->getOrigState().m_vel;
                if(distV != ngl::Vec3(0,0,0)){distV.normalize();}
                distV * m_agent->getOrigState().m_rad;
                m_agent->setVel(tmpVel+0.1*distV);
            }
        }
        separation += tmpSeparation;
    }
    BOOST_FOREACH(Boundary *b,m_Boundaries)
    {
        // find closest perpendicular point to boundary
        // use this distance for repulsion
        ngl::Vec3 tmpSeparation;
        // separation from neighbouring boids
        ngl::Vec3 bPoints1 = b->getBoundaryPoint(0);
        ngl::Vec3 bPoints2 = b->getBoundaryPoint(1);
        ngl::Vec3 bEdge1 = bPoints1 - bPoints2;
        ngl::Vec3 perpEdge = ngl::Vec3(-bEdge1.m_z,0,bEdge1.m_x);
        //ngl::Vec3 edge2 = m_agent->getOrigState().m_pos + perpEdge;

        // To find perpendicular point:
        // bounddary p+tr = q+us agent
        // t = (q − p) × s / (r × s)
        // if 0<t<1  point exists
        float t = (m_agent->getCurrentState().m_pos - bPoints2).cross(perpEdge).m_y /
                  (bEdge1.cross(perpEdge).m_y);
        // nearest point on boundary
        ngl::Vec3 iPoint = bPoints2 + (t*bEdge1);

        ngl::Vec3 distV = m_agent->getOrigState().m_pos - iPoint;

        float distF = (m_agent->getOrigState().m_pos - iPoint).length() -
                     (m_agent->getOrigState().m_rad);

        if(distF < m_perceiveRad)
        {
            tmpSeparation -= (iPoint - m_agent->getOrigState().m_pos);
            tmpSeparation *= (m_perceiveRad/distF);

            if(distF <= m_agent->getOrigState().m_rad)
            {
                ngl::Vec3 tmpVel = m_agent->getOrigState().m_vel;
                if(distV != ngl::Vec3(0,0,0)) {distV.normalize();}
                distV * m_agent->getOrigState().m_rad;
                m_agent->setVel(tmpVel+1*distV);
            }
        }
        separation += tmpSeparation;

    }

    if(separation != ngl::Vec3(0.0f,0.0f,0.0f))
    {
        separation.normalize();
        separation *= m_separationWeight;
    }

    return separation;
}

ngl::Vec3 Brain::flockCohesion()
{
    ngl::Vec3 cohesion = ngl::Vec3(0.0f,0.0f,0.0f);
    BOOST_FOREACH(Agent* n, m_neighbours)
    {
        cohesion += n->getOrigState().m_pos - m_agent->getOrigState().m_pos;
    }
    cohesion /= m_neighbours.size();
    if(cohesion!= ngl::Vec3(0.0f,0.0f,0.0f))
    {
        //cohesion - m_agent->getOrigState().m_pos;
        cohesion.normalize();
        cohesion *= m_cohesionWeight;
    }

    return cohesion;
}

ngl::Vec3 Brain::flockAlignment()
{
    ngl::Vec3 alignment;
    BOOST_FOREACH(Agent* n, m_neighbours)
    {
        alignment += n->getOrigState().m_vel;
    }
    if(alignment != ngl::Vec3(0.0f,0.0f,0.0f))
    {
        alignment.normalize();
        //alignment /= m_neighbours.size();
        alignment*= m_alignmentWeight;
    }

    return alignment;
}

ngl::Vec3 Brain::flockGoal()
{

}

//----------------Social forces-----------------------
void Brain::socialForces()
{
    //std::cout<<"social forces avoidance in use\n"<<std::endl;
}

//-----------------------------------------------------
float Brain::getPerceiveRad()const
{
    return m_perceiveRad;
}

float Brain::getPerceiveAng()const
{
    return m_perceiveAng;
}

float Brain::getDesSpeed()const
{
    return m_desSpeed;
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
    m_neighbours.clear();
}

void Brain::addNeighbour( Agent* _neighbour)
{
    m_neighbours.push_back(_neighbour);
}

void Brain::addBoundary(Boundary *_boundary)
{
    m_Boundaries.push_back(_boundary);
}
