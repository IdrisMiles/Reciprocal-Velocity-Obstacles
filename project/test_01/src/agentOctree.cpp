#include <boost/foreach.hpp>
#include "agentOctree.h"
#include "Agent.h"
#include <ngl/NGLStream.h>


void AgentOctree::checkAgentNeighbours(TreeNode<Agent> *node)
{
    if(node->m_height !=1)
    {
        for(int i=0;i<8;++i)
        {
            checkAgentNeighbours(node->m_child[i]);
        }
    }
    else
    {
        if(node->m_objectList.size()<=1)
        {
            return;
        }
        BOOST_FOREACH(boost::shared_ptr<Agent> currentAgent,node->m_objectList)
        {
            currentAgent->getBrain()->clearNeighbours();

            BOOST_FOREACH(boost::shared_ptr<Agent> testAgent,node->m_objectList )
            {
                // no need to self test
                if(testAgent==currentAgent)
                {
                  // continue to next foor loop iteration
                  continue;
                }
                float dist = (currentAgent->getOrigState().m_pos - testAgent->getOrigState().m_pos).length() -
                              (currentAgent->getOrigState().m_rad + testAgent->getOrigState().m_rad);
                if(dist <= currentAgent->getBrain()->getPerceiveRad())
                  {
                    // add agent to neighbours if within perceive radius
                    currentAgent->getBrain()->addNeighbour(testAgent);
                  }
            }
        }
    }



  return;
}
