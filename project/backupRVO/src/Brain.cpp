#include "Brain.h"

#include <iostream>
#include <math.h>
#include <boost/foreach.hpp>
#include <ngl/NGLStream.h>
#include <ngl/Transformation.h>
#include <ngl/NGLStream.h>
#include <ngl/ShaderLib.h>

#include "Agent.h"

#define MAX 1000000000

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

    m_socialAttraction = 0;

}

Brain::~Brain()
{
    if(m_voVAO != NULL){m_voVAO->removeVOA();}
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
            initVoVAO();
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
  // use social forces to find the next desired position.

    m_desVel = m_goal - m_agent->getOrigState().m_pos;
    if(m_desVel != ngl::Vec3(0,0,0))
    {
        m_desVel.normalize();
    }
    m_desVel *= m_desSpeed;

    ngl::Vec3 socialForce = (m_desVel - m_agent->getOrigState().m_vel);
    ngl::Vec3 repulseForce = ngl::Vec3(0.0f,0.0f,0.0f);

    if(m_neighbours.size() == 0){return;}

    BOOST_FOREACH(Agent *a, m_neighbours)
    {
      repulseForce += (m_agent->getOrigState().m_pos - a->getOrigState().m_pos);
    }
    repulseForce /= m_neighbours.size();
    socialForce += repulseForce;

    float socialFactor = m_socialAttraction * (m_agent->getOrigState().m_pos - m_goal).lengthSquared();
    m_desVel += socialFactor * socialForce;

}

void Brain::setSocialAttraction(const float &_socialAttract)
{
  m_socialAttraction = _socialAttract;
}

//===============RVO==================
void Brain::initVoVAO()
{
    glPointSize(5);
    m_voVAO = ngl::VertexArrayObject::createVOA(GL_POINTS);
    m_voVAO->bind();
    std::vector<ngl::Vec3> voData;
    for(int i=0;i<1;i++)
    {
        voData.push_back(m_agent->getOrigState().m_pos);
        voData.push_back(m_agent->getOrigState().m_pos);
        voData.push_back(m_agent->getOrigState().m_pos);
    }
    m_voVAO->setData(voData.size()*sizeof(ngl::Vec3),voData[0].m_x);
    m_voVAO->setVertexAttributePointer(0,3,GL_FLOAT,0,0);
    m_voVAO->setNumIndices(10*voData.size());
    m_voVAO->unbind();
}


void Brain::rvo()
{
    std::vector<ngl::Vec3> testVelocities;
    testVelocities = createSampleVel();

    ngl::Vec3 newVel = m_desVel;
    newVel = findNewVelRVO(testVelocities);

    m_agent->setVel(newVel);

}

ngl::Vec3 Brain::findNewVelRVO(const std::vector<ngl::Vec3> &testVelocities)
{
    std::vector<float> tValues;
    ngl::Vec3 newVel = m_desVel;
    bool velInside = true;
    for(unsigned int i=0; i<testVelocities.size() && velInside;i++)
    {
        newVel = testVelocities[i];
        std::vector<bool> velAcceptance;
        std::vector<float> agentTvalues;

        BOOST_FOREACH(Agent *n, m_neighbours)
        {
            // check if velocity is acceptable with current test agent
            velAcceptance.push_back(testVO(newVel,n->getOrigState(),agentTvalues));

        } // end of neighbour boost foreach loop

        BOOST_FOREACH(Boundary *b, m_Boundaries)
        {
            // check if velocity is acceptable with current test agent
            velAcceptance.push_back(testVO(newVel,b,agentTvalues));

        } // end of neighbour boost foreach loop


        // check if all elements of velAccepted are true
        // if so current test velocity is outside all VO's
        // we can break out of testVelocities for loop
        bool accept = true;
        for(unsigned int k=0;k<velAcceptance.size() && accept;k++)
        {
            if(!velAcceptance[k])
            {
                accept = false;
            }
        }
        // not necessarily needed?
        if(accept)
        {
            // velocity is outside VO, thus is acceptable
            velInside = false;
        }
        else
        {
            // store T value
            float lowestT = MAX;
            for(unsigned int k=0;k<agentTvalues.size();k++)
            {
                if(agentTvalues[k]<lowestT && lowestT>0)
                {
                    lowestT = agentTvalues[k];
                }
            }
            tValues.push_back(lowestT);
        }

    } // end of testVelocities for loop

    // if acceptable velocity was found exit now
    if(!velInside){return newVel;}

    // work out penalty for velocities
    int index=-1;
    float penalty = MAX;
    float t = 1;
    for(unsigned int i=0;i<tValues.size();i++)
    {
        float tmpPenalty = (10/tValues[i]);// + (m_desVel-testVelocities[i]).length();
        if(tmpPenalty < penalty && tmpPenalty>0)
        {
            penalty = tmpPenalty;
            t = tValues[i];
            index = i;
        }
    }
    // set new velocity
    //newVel = 0.5 * (t*testVelocities[index] + m_agent->getOrigState().m_vel);
    newVel = t*testVelocities[index];
    return newVel;
}

bool Brain::testVO(const ngl::Vec3 &_testVel, const State &_testAgentState, std::vector<float> &_agentTvalues)
{
    ngl::Vec3 dist = _testAgentState.m_pos  - m_agent->getOrigState().m_pos;
    // vector orthogonal to dist vector
    ngl::Vec3 distNormal = normal2D(dist);
    distNormal.normalize();

    // cobined radius of current agent and test agent
    float r1 = m_agent->getOrigState().m_rad;
    float r2 = _testAgentState.m_rad;
    float rad = r1+r2;

    // velocity obstacle apex offset
    ngl::Vec3 apexOffset = 0.5 * (m_agent->getOrigState().m_vel + _testAgentState.m_vel);

    // 3 points of Velocity Obstacle triangle - this is my own simplification
    // p1 is the apex of the VO
    ngl::Vec3 p1 = m_agent->getOrigState().m_pos + apexOffset;
    ngl::Vec3 p2 = _testAgentState.m_pos + (rad * distNormal) + apexOffset;
    ngl::Vec3 p3 = _testAgentState.m_pos - (rad * distNormal) + apexOffset;

    // 3 edges of Velocity Obstacle triangle
    ngl::Vec3 E1 = p3 - p2; //back of collision area  - 2 * opposite
    ngl::Vec3 E2 = p1 - p3; //one side                - hypotonuse
    ngl::Vec3 E3 = p2 - p1; //other side

    // find length to bottom of VO
    float A = (m_agent->getOrigState().m_pos - _testAgentState.m_pos).length();
    float H = E2.length();
    float newH = (A - rad) * tan(asin(rad / H));

    E2.normalize();
    E2 *= newH;
    E3.normalize();
    E3 *= newH;

    ngl::Vec3 p4 = p1 + E3;
    ngl::Vec3 p5 = p1 - E2;

    E1 = p5 - p4;
    ngl::Vec3 leftEdge = p2 - p4;
    ngl::Vec3 rightEdge = p3 - p5;
    ngl::Vec3 frontEdge = p5 - p4;

    bool leftPlane = pointLeftOfEdge(m_agent->getOrigState().m_pos + _testVel, leftEdge,p4);
    bool rightPlane = !pointLeftOfEdge(m_agent->getOrigState().m_pos + _testVel, rightEdge,p5);
    bool frontPlane = pointLeftOfEdge(m_agent->getOrigState().m_pos + _testVel, frontEdge,p4);

    if(leftPlane || rightPlane || frontPlane)
    {
        // tested velcity is acceptable!
        return true;
    }
    else

    {
        // tested velocity inside VO - not acceptable, will result in collision
        // need to find the smallest positive value for t - time to intersection

        float time = 2*m_perceiveRad / _testVel.length();

        // check if test velocity intersects multipe edges of the VO triangle
        float i1 = checkIntersection(_testVel,m_agent->getOrigState().m_pos,frontEdge,p4,time);
        float i2 = checkIntersection(_testVel,m_agent->getOrigState().m_pos,rightEdge,p5,time);
        float i3 = checkIntersection(_testVel,m_agent->getOrigState().m_pos,leftEdge,p4,time);

        float tmp1 = (i1>0)?i1:MAX;
        float tmp2 = (i2>0)?i2:MAX;
        float tmp3 = (i3>0)?i3:MAX;

        float lowestT   = (tmp1<tmp2)   ?tmp1:tmp2;
        lowestT         = (tmp3<lowestT)?tmp3:lowestT;

        _agentTvalues.push_back(lowestT);
        return false;
    }
}

bool Brain::testVO(const ngl::Vec3 &_testVel, const Boundary *n, std::vector<float> &_agentTvalues)
{
    // get the boundary points
    ngl::Vec3 p0 = n->getBoundaryPoint(0);
    ngl::Vec3 p1 = n->getBoundaryPoint(1);
    ngl::Vec3 p2 = n->getBoundaryPoint(2);
    ngl::Vec3 p3 = n->getBoundaryPoint(3);

    ngl::Vec3 centre = 0.25 * (p0 + p1 + p2 + p3);

    // new boundary points taking into account agent radius
    ngl::Vec3 tmp0 = (m_agent->getOrigState().m_pos - p0);
    ngl::Vec3 tmp1 = (m_agent->getOrigState().m_pos - p1);
    ngl::Vec3 tmp2 = (m_agent->getOrigState().m_pos - p2);
    ngl::Vec3 tmp3 = (m_agent->getOrigState().m_pos - p3);

    tmp0.normalize();
    tmp1.normalize();
    tmp2.normalize();
    tmp3.normalize();



    ngl::Vec3 np0 = p0 + (m_agent->getOrigState().m_rad)*tmp0;
    ngl::Vec3 np1 = p1 + (m_agent->getOrigState().m_rad)*tmp1;
    ngl::Vec3 np2 = p2 + (m_agent->getOrigState().m_rad)*tmp2;
    ngl::Vec3 np3 = p3 + (m_agent->getOrigState().m_rad)*tmp3;

    // create boundary edges
    ngl::Vec3 e0 = np0 - np1;
    ngl::Vec3 e1 = np1 - np2;
    ngl::Vec3 e2 = np2 - np3;
    ngl::Vec3 e3 = np3 - np0;

    // test whether the velocity lies outside of the boundary
    bool leftPlane  = pointLeftOfEdge(m_agent->getOrigState().m_pos + _testVel,e0,np1);
    bool rightPlane = pointLeftOfEdge(m_agent->getOrigState().m_pos + _testVel,e1,np2);
    bool frontPlane = pointLeftOfEdge(m_agent->getOrigState().m_pos + _testVel,e2,np3);
    bool backPlane  = pointLeftOfEdge(m_agent->getOrigState().m_pos + _testVel,e3,np0);

    if(leftPlane || rightPlane || frontPlane || backPlane)
    {
        // tested velcity is acceptable!
        return true;
    }
    else
    {
        float time = 2*m_perceiveRad / _testVel.length();

        // find the time to collision
        float i1 = checkIntersection(_testVel,m_agent->getOrigState().m_pos,e0,np0,time);
        float i2 = checkIntersection(_testVel,m_agent->getOrigState().m_pos,e1,np1,time);
        float i3 = checkIntersection(_testVel,m_agent->getOrigState().m_pos,e2,np2,time);
        float i4 = checkIntersection(_testVel,m_agent->getOrigState().m_pos,e3,np3,time);

        float tmp1 = (i1>0)?i1:MAX;
        float tmp2 = (i2>0)?i2:MAX;
        float tmp3 = (i3>0)?i3:MAX;
        float tmp4 = (i4>0)?i4:MAX;

        float lowestT   = (tmp1<tmp2)   ?tmp1:tmp2;
              lowestT   = (tmp3<lowestT)?tmp3:lowestT;
              lowestT   = (tmp4<lowestT)?tmp4:lowestT;

        _agentTvalues.push_back(lowestT*0.0001);
        return false;
    }

}

std::vector<ngl::Vec3> Brain::createSampleVel()const
{
    std::vector<ngl::Vec3> testVelocities;
    testVelocities.push_back(m_desVel);
    ngl::Vec3 tmpVel = m_desVel;

    int numSamples = 36;
    for(int i=0;i<numSamples;i++)
    {
        ngl::Mat4 rot;
        rot.rotateY((i*180)/numSamples);
        tmpVel = matXvec(rot,m_desVel);
        testVelocities.push_back(tmpVel);

        rot.identity();
        rot.rotateY((-i*170)/numSamples);
        tmpVel = matXvec(rot,m_desVel);
        testVelocities.push_back(tmpVel);

    }

    return testVelocities;
}

bool Brain::pointLeftOfEdge(const ngl::Vec3 &_point,
                     const ngl::Vec3 &_edge, const ngl::Vec3 &_edgePoint)const
{
    ngl::Vec3 tmp = _point - _edgePoint;
    float result = det2D(_edge,tmp);
    return (result >= 0);
}

float Brain::checkIntersection(const ngl::Vec3 &_vel, const ngl::Vec3 &_p1, const ngl::Vec3 &_edge, const ngl::Vec3 &_p2, const float &_time)const
{
    // return -1 when no collision
    //Velocity Ray: P + t*r  Triangle Edge: Q + u*s
    //t = (q − p) × s / (r × s)
    //u = (q − p) × r / (r × s)
    // replace with desired direction
    float t,u;
    float dirCross = det2D(_vel,_edge);
    if(dirCross != 0)
    {
        // two agents will collide
        ngl::Vec3 dist = _p2 - _p1;
        float tCross = det2D(dist,_edge);
        float uCross = det2D(dist,_vel);

        t = -tCross / dirCross;
        u = -uCross / dirCross;
        //std::cout<<"collision will occur in"<<t<<"\n";


        if(t>=0 && t<=_time)
        {
           // std::cout<<"collision in "<<t<<"\n";
            return t;
            // Agent is going to collide with edge in t amount of time
        }
        //else {return -1;}
    }

    return -1;
}

float Brain::det2D(const ngl::Vec3 &_v1, const ngl::Vec3 &_v2) const
{
    return (_v1.m_x * _v2.m_z) - (_v1.m_z * _v2.m_x);
}

ngl::Vec3 Brain::normal2D(const ngl::Vec3 &_vec)const
{
    ngl::Mat4 rot;
    rot.identity();
    rot.rotateY(90);

    ngl::Vec3 tmp;
//    tmp.m_x = _vec.m_y;
//    tmp.m_y = -_vec.m_x;

    tmp = matXvec(rot,_vec);
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

//=============flocking=========================================
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
        float t = (m_agent->getOrigState().m_pos - bPoints2).cross(perpEdge).m_y /
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

ngl::Vec3 Brain::getDesVel()const
{
  return m_desVel;
}


void Brain::clearNeighbours()
{
    m_neighbours.clear();
    m_neighbourIDs.clear();
}

void Brain::clearBoundary()
{
    m_Boundaries.clear();
}

void Brain::addNeighbour( Agent* _neighbour)
{
    m_neighbours.push_back(_neighbour);
    m_neighbourIDs.push_back(_neighbour->getID());
}

void Brain::addBoundary(Boundary *_boundary)
{
    m_Boundaries.push_back(_boundary);
}

std::vector<Boundary*> Brain::getBoundaries()const
{
    return m_Boundaries;
}

void Brain::printInfo()
{

}
