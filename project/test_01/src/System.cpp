#include "System.h"

#include <ngl/Random.h>
#include <boost/foreach.hpp>

System::System()
{
    m_spatialDivision = BRUTE;

    BoundingBox bb;
    bb.m_minx = bb.m_miny = bb.m_minz = -10.0;
    bb.m_maxx = bb.m_maxy = bb.m_maxz = 10.0;
    m_octree = new AgentOctree (3, bb);

}

System::~System()
{

}

void System::update()
{
    addNeighbours();
    addBoundaries();

    BOOST_FOREACH( boost::shared_ptr<Agent> a, m_agents)
    {
        a->update();
    }

    BOOST_FOREACH( boost::shared_ptr<Agent> a, m_agents)
    {
        a->updateState();
    }

    clearNeighbours();
    clearBoundaries();
}

void System::addAgent(const Avoidance &_avoidType)
{
    boost::shared_ptr<Agent> tAgent(new Agent(this,_avoidType));
    m_agents.push_back(tAgent);
}

void System::addNeighbours()
{
  switch (m_spatialDivision)
    {
    case BRUTE :
      BOOST_FOREACH( boost::shared_ptr<Agent> a, m_agents)
      {
          BOOST_FOREACH( boost::shared_ptr<Agent> b, m_agents)
          {
              if(a == b){continue;}
              a->getBrain()->addNeighbour(b);
          }
      }
      break;

    case OCTREE :
     m_octree->clearTree();
      BOOST_FOREACH( boost::shared_ptr<Agent> a, m_agents)
      {
          m_octree->addObject(a);
      }
      m_octree->addNeighbours();
      break;
    }
}

void System::addBoundaries()
{
    switch (m_spatialDivision)
      {
      case BRUTE :
        BOOST_FOREACH( boost::shared_ptr<Agent> a, m_agents)
        {
            BOOST_FOREACH(Boundary *b, m_Boundaries)
            {
                // check distance to boundary
                // add if within perceive rad
                a->getBrain()->addBoundary(b);
            }
        }
        break;

      case OCTREE :
       m_octree->clearTree();
        BOOST_FOREACH( boost::shared_ptr<Agent> a, m_agents)
        {
            m_octree->addObject(a);
        }
        m_octree->addNeighbours();
        break;
      }

}

void System::clearNeighbours()
{
    BOOST_FOREACH( boost::shared_ptr<Agent> a, m_agents)
    {
        a->getBrain()->clearNeighbours();
    }
}

void System::clearBoundaries()
{
    BOOST_FOREACH( boost::shared_ptr<Agent> a, m_agents)
    {
        a->getBrain()->clearBoundary();
    }
}

void System::setSpatialDivision(const SpatialDivision &_type)
{
  m_spatialDivision = _type;
}

void System::setGloablGoal(const ngl::Vec3 &_goal)
{
    BOOST_FOREACH(boost::shared_ptr<Agent> a,m_agents)
    {
        a->getBrain()->setGoal(_goal);

//        ngl::Random *r = ngl::Random::instance();
//        a->getBrain()->setGoal(r->getRandomVec3()*10);
    }
}

void System::draw()
{
    BOOST_FOREACH(boost::shared_ptr<Agent> a, m_agents)
    {
        a->loadMatricesToShader();
        a->draw();
    }
}

void System::setUpDraw(const ngl::Camera &_cam, const ngl::Mat4 &_tx)
{
    m_cam = _cam;
    m_globalTX = _tx;
    //m_globalTX;
}

ngl::Camera System::getCam()const
{
    return m_cam;
}
ngl::Mat4 System::getGlobalTX()const
{
    return m_globalTX;
}



