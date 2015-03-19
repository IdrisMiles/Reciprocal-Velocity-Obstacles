#include "System.h"

#include <ngl/Random.h>
#include <ngl/ShaderLib.h>
#include <ngl/Mat4.h>
#include <boost/foreach.hpp>

#include "HashTable.h"

System::System()
{
    //m_bounds = ngl::BBox(ngl::Vec3(0,0,0),20,20,20);
    BoundingBox bb;
    bb.m_minx = bb.m_miny = bb.m_minz = -10.0;
    bb.m_maxx = bb.m_maxy = bb.m_maxz = 10.0;

    setBounds(ngl::BBox(ngl::Vec3(0,5,0),20,10,20));
    m_spatialDivision = BRUTE;

    m_octree = new AgentOctree (3, bb);
    m_hashTable = new HashTable(20,20,1);



    m_globalGoal = ngl::Vec3(0.0f,0.0f,0.0f);
}

System::~System()
{
    delete [] m_octree;
    delete [] m_hashTable;

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
    m_agents.back()->getBrain()->setGoal(m_globalGoal);
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

    case HASH :
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

    case HASH :
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
    //m_octree = new AgentOctree (3, bb);
  m_spatialDivision = _type;
}

void System::setBounds(ngl::BBox _bounds)
{
    m_bounds = _bounds;

    float left,right,bottom,top;
    left   = m_bounds.center().m_x - (0.5*m_bounds.width());
    right  = m_bounds.center().m_x + (0.5*m_bounds.width());
    bottom = m_bounds.center().m_z - (0.5*m_bounds.depth());
    top    = m_bounds.center().m_z + (0.5*m_bounds.depth());

    ngl::Vec3 BL,TL,BR,TR;
    BL = ngl::Vec3(left, 0,bottom);
    TL = ngl::Vec3(left, 0,top);
    BR = ngl::Vec3(right,0,bottom);
    TR = ngl::Vec3(right,0,top);

    // left side
    m_Boundaries.push_back(new Boundary(BL,TL));
    // right side
    m_Boundaries.push_back(new Boundary(TR,BR));
    // top side
    m_Boundaries.push_back(new Boundary(TL,TR));
    // bottom side
    m_Boundaries.push_back(new Boundary(BR,BL));
}

void System::setGloablGoal(const ngl::Vec3 &_goal)
{
    m_globalGoal = _goal;
    BOOST_FOREACH(boost::shared_ptr<Agent> a,m_agents)
    {
        a->getBrain()->setGoal(_goal);
    }
}

void System::setRandomGoal()
{
    BOOST_FOREACH(boost::shared_ptr<Agent> a,m_agents)
    {
        ngl::Random *r = ngl::Random::instance();
        ngl::Vec3 goal = r->getRandomVec3()*10;
        a->getBrain()->setGoal(goal);
    }
}

void System::loadMatricesToShader()
{
  ngl::ShaderLib *shader=ngl::ShaderLib::instance();

  ngl::Mat4 MV;
  ngl::Mat4 MVP;
  ngl::Mat3 normalMatrix;
  ngl::Mat4 M;

  M   = m_globalTX;
  MV  = M*m_cam.getViewMatrix();
  MVP = M*m_cam.getVPMatrix();
  normalMatrix=MV;
  normalMatrix.inverse();

  shader->setShaderParamFromMat4("MV",MV);
  shader->setShaderParamFromMat4("MVP",MVP);
  shader->setShaderParamFromMat3("normalMatrix",normalMatrix);
  shader->setShaderParamFromMat4("M",M);
}

void System::draw()
{
    loadMatricesToShader();
    m_bounds.recalculate();
    m_bounds.draw();
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



