#include "System.h"

#include <boost/foreach.hpp>

System::System()
{
    BoundingBox bb;
    bb.m_minx = bb.m_miny = bb.m_minz = -5.0;
    bb.m_maxx = bb.m_maxy = bb.m_maxz = 5.0;
    m_octree = new AgentOctree (3, bb);

}

System::~System()
{

}

void System::update()
{
    addNeighbours();

    BOOST_FOREACH(Agent *a, m_agents)
    {
        a->update();
    }
}

void System::addAgent(const Avoidance &_avoidType)
{
    m_agents.push_back(new Agent(this,_avoidType));
}

void System::addNeighbours()
{
    m_octree->clearTree();

    BOOST_FOREACH(Agent *a, m_agents)
    {
        m_octree->addObject(a);
    }
    m_octree->addNeighbours();
}

//void System::addBoundaries()
//{
//    m_octree->clearTree();


//    BOOST_FOREACH(Agent *a, m_agents)
//    {
//        m_octree->addObject(a);
//    }
//    m_octree->add
//}

void System::draw()
{
    BOOST_FOREACH(Agent *a, m_agents)
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



