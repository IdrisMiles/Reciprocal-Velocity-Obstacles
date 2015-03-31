#include "Brain.h"

#include <iostream>
#include <math.h>
#include <boost/foreach.hpp>
#include <ngl/NGLStream.h>
#include <ngl/Transformation.h>

#include "Agent.h"

Brain::Brain(Agent *_agent, System *_system)
{
    // ctor that should be used
    m_agent = _agent;
    m_system = _system;

    m_perceiveRad = 1.0f;
    m_perceiveAng = 360.0f;
    m_desSpeed = 0.1;
    m_maxVel = 0.2;
    m_maxAcc = 0.1;
    m_goal = -m_agent->getCurrentState().m_pos;

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
    if(m_desVel != ngl::Vec3(0,0,0))
    {
        m_desVel.normalize();
    }
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
    // ------TO DO------
    /*
     * find velocities based on desired velocity to sample - 10(for now)
     * iterate through velocities and either:
     *    o find the first velocity that lies outside VO
     *    o find velocity with smallestt penalty
     * To find velocity with smallest penalty:
     *    o find velocity with longest time until collision - t
     *    o take into account distance between chosen velocity and desired velocity.
    */

    std::vector<ngl::Vec3> testVelocities;
    testVelocities.push_back(m_desVel);
    ngl::Vec3 tmpVel = m_desVel;
    tmpVel.normalize();
    ngl::Vec3 minVel = 0.05 * tmpVel;
    ngl::Vec3 maxVel = m_maxVel * tmpVel;
    testVelocities.push_back(maxVel);
    testVelocities.push_back(minVel);


    for(int i=0;i<10;i++)
    {
        ngl::Mat4 rot;
        rot.rotateY((i*180)/10);
        tmpVel = matXvec(rot,m_desVel);
        // push back test velocities to sample
        testVelocities.push_back(tmpVel);

        rot.identity();
        rot.rotateY((-i*180)/10);
        tmpVel = matXvec(rot,m_desVel);
        testVelocities.push_back(tmpVel);
    }

    ngl::Vec3 newVel = m_desVel;
    bool velInside = true;
    for(unsigned int i=0; i<testVelocities.size() && velInside;i++)
    {
        newVel = testVelocities[i];
        int j = 0;
        std::vector<bool> velAcceptance;
        BOOST_FOREACH(Agent *n, m_neighbours)
        {
            j++;
            // vector between current agent and test agent
            ngl::Vec3 dist = n->getOrigState().m_pos  - m_agent->getOrigState().m_pos;
            // vector orthogonal to dist vector
            ngl::Vec3 distNormal = normal2D(dist);
            distNormal.normalize();

            // cobined radius of current agent and test agent
            float r1 = m_agent->getOrigState().m_rad;
            float r2 = n->getOrigState().m_rad;
            float rad = r1+r2;

            // velocity obstacle apex offset
            ngl::Vec3 apexOffset = 0.5 * (m_agent->getOrigState().m_vel + n->getOrigState().m_vel);

            // 3 points of Velocity Obstacle triangle - this is my own simplification
            // p1 is the apex of the VO
            ngl::Vec3 p1 = m_agent->getOrigState().m_pos + apexOffset;
            ngl::Vec3 p2 = n->getOrigState().m_pos + apexOffset + (rad * distNormal);
            ngl::Vec3 p3 = n->getOrigState().m_pos + apexOffset - (rad * distNormal);

            // 3 edges of Velocity Obstacle triangle
            ngl::Vec3 E1 = p3 - p2;
            ngl::Vec3 E2 = p2 - p1;
            ngl::Vec3 E3 = p1 - p3;

            // heuristic to find test velocity
            ngl::Vec3 testVel = m_desVel;

            // check if test velocity intersects multipe edges of the VO triangle
            float i1 = checkIntersection(testVel,m_agent->getOrigState().m_pos,E1,p1);
            float i2 = checkIntersection(testVel,m_agent->getOrigState().m_pos,E2,p2);
            float i3 = checkIntersection(testVel,m_agent->getOrigState().m_pos,E3,p3);

            // if only 1 intersection, test velocity inside VO
            // if 0 or 2 intersections, test velocity outside VO - acceptable velocity
            if( (i1 != -1 && i2 != -1) ||
                (i2 != -1 && i3 != -1) ||
                (i3 != -1 && i1 != -1) ||
                (i1 == -1 && i2 == -1 && i3 == -1))
            {
                // tested velcity is acceptable!
                velAcceptance.push_back(true);

            }
            else
            {
                // tested velocity inside VO - not acceptable, will result in collision
                newVel = ngl::Vec3(0,0,0);
                velAcceptance.push_back(false);
                // could break neighbour foreach loop and go to next test velocity
            }
        } // end of neighbour boost foreach loop

        if(j==m_neighbours.size())
        {
            // check if all elements of velAccepted are true
            // if so current test velocity is outside all VO's
            // we can break out of testVelocities for loop
            //velInside = false;
            bool accept = true;
            for(int k=0;k<velAcceptance.size();k++)
            {
                if(!velAcceptance[i])
                {
                    accept = false;
                }
            }

        }

    } // end of testVelocities for loop


//    BOOST_FOREACH(Agent *n, m_neighbours)
//    {
        //----------------tmp shizzle------------------------
        /*float t,u;
        //Velocity Ray: P + t*r  Triangle Edge: Q + u*s
        //t = (q − p) × s / (r × s)
        //u = (q − p) × r / (r × s)
        // replace with desired direction
        float dirCross = det2D(m_agent->getOrigState().m_vel,n->getOrigState().m_vel);
        if(dirCross != 0)
        {
            ngl::Vec3 dist = n->getOrigState().m_pos - m_agent->getOrigState().m_pos;
            float tCross = det2D(dist,n->getOrigState().m_vel);
            float uCross = det2D(dist,m_agent->getOrigState().m_vel);

            t = tCross / dirCross;
            u = uCross / dirCross;

            // two agents will collide
            if(t>=0 && t<=time)
            {
                //std::cout<<"collision in "<<t<<" time frame\n";
                // Agent is going to collide with neighbour in t amount of time
            }
        }*/

        //-------------actuall implementation-------------------

/*
        // vector between current agent and test agent
        ngl::Vec3 dist = n->getOrigState().m_pos  - m_agent->getOrigState().m_pos;
        // vector orthogonal to dist vector
        ngl::Vec3 distNormal = normal2D(dist);
        distNormal.normalize();

        // cobined radius of current agent and test agent
        float r1 = m_agent->getOrigState().m_rad;
        float r2 = n->getOrigState().m_rad;
        float rad = r1+r2;

        // velocity obstacle apex offset
        ngl::Vec3 apexOffset = 0.5 * (m_agent->getOrigState().m_vel + n->getOrigState().m_vel);

        // 3 points of Velocity Obstacle triangle - this is my own simplification
        // p1 is the apex of the VO
        ngl::Vec3 p1 = m_agent->getOrigState().m_pos + apexOffset;
        ngl::Vec3 p2 = n->getOrigState().m_pos + apexOffset + (rad * distNormal);
        ngl::Vec3 p3 = n->getOrigState().m_pos + apexOffset - (rad * distNormal);

        // 3 edges of Velocity Obstacle triangle
        ngl::Vec3 E1 = p3 - p2;
        ngl::Vec3 E2 = p2 - p1;
        ngl::Vec3 E3 = p1 - p3;

        // heuristic to find test velocity
        ngl::Vec3 testVel = m_desVel;

        // check if test velocity intersects multipe edges of the VO triangle
        float i1 = checkIntersection(testVel,m_agent->getOrigState().m_pos,E1,p1);
        float i2 = checkIntersection(testVel,m_agent->getOrigState().m_pos,E2,p2);
        float i3 = checkIntersection(testVel,m_agent->getOrigState().m_pos,E3,p3);

        // if only 1 intersection, test velocity inside VO
        // if 0 or 2 intersections, test velocity outside VO - acceptable velocity
        if( (i1 != -1 && i2 != -1) ||
            (i2 != -1 && i3 != -1) ||
            (i3 != -1 && i1 != -1) ||
            (i1 == -1 && i2 == -1 && i3 == -1))
        {
            // tested velcity is acceptable!
        }
        else
        {
            // tested velocity inside VO - not acceptable, will result in collision
            newVel = ngl::Vec3(0,0,0);
        }

    }
    */


    // set new velocity
    m_agent->setVel(newVel);

}

float Brain::checkIntersection(const ngl::Vec3 &_vel, const ngl::Vec3 &_p1, const ngl::Vec3 &_edge, const ngl::Vec3 &_p2)const
{
    //Velocity Ray: P + t*r  Triangle Edge: Q + u*s
    //t = (q − p) × s / (r × s)
    //u = (q − p) × r / (r × s)
    // replace with desired direction
    float t;
    float time = 1.0f;
    float dirCross = det2D(_vel,_edge);
    if(dirCross != 0)
    {
        // two agents will collide
        ngl::Vec3 dist = _p2 - _p1;
        float tCross = det2D(dist,_edge);
        float uCross = det2D(dist,_vel);

        t = tCross / dirCross;
        //u = uCross / dirCross;


        if(t>=0 && t<=time)
        {
            return t;
            // Agent is going to collide with edge in t amount of time
        }
    }
    return -1;
}

float Brain::det2D(const ngl::Vec3 &_v1, const ngl::Vec3 &_v2) const
{
    return (_v1.m_x * _v2.m_z) - (_v1.m_z * _v2.m_x);
}

ngl::Vec3 Brain::normal2D(const ngl::Vec3 &_vec)const
{
    ngl::Vec3 tmp = ngl::Vec3(0,0,0);
    tmp.m_x = _vec.m_y;
    tmp.m_y = -_vec.m_x;
    return tmp;
}

ngl::Vec3 Brain::matXvec(const ngl::Mat4 &_mat, const ngl::Vec3 &_pos)const
{
    ngl::Vec4 output;
      for(int i=0;i<4;i++)
      {
          for(int j=0;j<4;j++)
          {
              output.m_openGL[i] += _mat.m_m[i][j] * _pos.m_openGL[j];
          }
      }

      return ngl::Vec3(output.m_x,output.m_y,output.m_z);
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


void Brain::printInfo()
{
  std::cout<<"num neighbours: "<<m_neighbours.size()<<"\n";
}
