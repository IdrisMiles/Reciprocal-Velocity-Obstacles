#ifndef AgentOCTRRE_H__
#define AgentOCTRRE_H__

#include "AbstractOctree.h"
#include "Agent.h"
//class Agent;

class AgentOctree : public AbstractOctree <Agent,ngl::Vec3>
{
  public :
     AgentOctree(int _height, BoundingBox _limit)
       :
       AbstractOctree <Agent,ngl::Vec3> ( _height,  _limit)
     {}

     virtual void  checkAgentNeighbours(TreeNode <Agent> *_node);

};

#endif

