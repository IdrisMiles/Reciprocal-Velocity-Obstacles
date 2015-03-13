#include "System.h"

#include <boost/foreach.hpp>

System::System()
{
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
//    BOOST_FOREACH( boost::shared_ptr<Agent> a, m_agents)
//    {
//        BOOST_FOREACH( boost::shared_ptr<Agent> b, m_agents)
//        {
//            if(a == b){continue;}
//            a->getBrain()->addNeighbour(b);
//        }
//    }

    BOOST_FOREACH( boost::shared_ptr<Agent> a, m_agents)
    {
        a->update();
    }

    BOOST_FOREACH( boost::shared_ptr<Agent> a, m_agents)
    {
        a->updateState();
        a->getBrain()->clearNeighbours();
        //a->getBrain()->clearBoundary();
    }
}

void System::addAgent(const Avoidance &_avoidType)
{
    boost::shared_ptr<Agent> tAgent(new Agent(this,_avoidType));
    //m_agents.push_back(new Agent(this,_avoidType));
    m_agents.push_back(tAgent);
}

void System::addNeighbours()
{
    m_octree->clearTree();

    BOOST_FOREACH( boost::shared_ptr<Agent> a, m_agents)
    {
        m_octree->addObject(a);
    }
    m_octree->addNeighbours();
}

//void System::addBoundaries()
//{
//    BOOST_FOREACH(Agent *a, m_agents)
//    {
//        BOOST_FOREACH(Boundary *b, m_Boundaries)
//        {
//            // check distance to boundary
//            // add if within perceive rad
//        }
//    }

//}


void System::setGloablGoal(const ngl::Vec3 &_goal)
{
    BOOST_FOREACH(boost::shared_ptr<Agent> a,m_agents)
    {
        a->getBrain()->setGoal(_goal);
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



