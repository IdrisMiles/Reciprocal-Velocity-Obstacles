#include "HashTable.h"
#include <iostream>

HashTable::HashTable(int _width, int _height, float _cellSize, const ngl::Vec3 &_centre)
{
  m_isVAOinit = false;
    m_width = _width;
    m_height = _height;
    m_cellSize = _cellSize;
    m_centre = _centre;

    m_numXcells = ceil((float)(m_width/m_cellSize));
    m_numYcells = ceil((float)(m_height/m_cellSize));

    m_cells.resize(m_numYcells * m_numXcells);
}

HashTable::HashTable(const ngl::BBox &_bound, int _cellSize)
{
  m_isVAOinit = false;
}

HashTable::~HashTable()
{
  m_vao->removeVOA();
  m_cellVao->removeVOA();
}


void HashTable::addAgent(Agent* _agent)
{
    // find cell agent should be in according to it's position
    Cell *tmpCell = getCell(*_agent);

    // add agent to cell
    tmpCell->m_agents.push_back(_agent);

    // update cell in agent
    _agent->setCell(tmpCell);

    // update agents cellID
    _agent->setCellID(tmpCell->m_agents.size() - 1);
}

void HashTable::addAgent(Agent* _agent, Cell* _cell)
{
    // add agent to cell
    _cell->m_agents.push_back(_agent);

    // update cell in agent
    _agent->setCell(_cell);

    // update agents cellID
    _agent->setCellID(_cell->m_agents.size() - 1);
}

void HashTable::removeAgent(Agent* _agent)
{
    // not in a cell
    if(_agent->getCellID() == -1){return;}

    // swap current agent with agent at back of cell
    _agent->getCell()->m_agents[_agent->getCellID()] = _agent->getCell()->m_agents.back();
    //std::swap(_agent->getCell()->m_agents[_agent->getCellID()],_agent->getCell()->m_agents.back());

    // push last element off
    _agent->getCell()->m_agents.pop_back();

    // if the agent was not the only agent in the cell
    if(_agent->getCell()->m_agents.size() != 0)
    {
        // update swapped agents cellID
        _agent->getCell()->m_agents[_agent->getCellID()]->setCellID(_agent->getCellID());
    }

    // update current agents cellID
    _agent->setCellID(-1);

}

void HashTable::emptyAgents()
{
    for(unsigned int i=0;i<m_cells.size();i++)
    {
        m_cells[i].m_agents.clear();
    }
}

void HashTable::emptyTable()
{
    for(unsigned int i=0;i<m_cells.size();i++)
    {
        m_cells[i].m_agents.clear();
    }
}

void HashTable::addBoundaryToHash(Boundary* _boundary)
{
  int startX = (int)(_boundary->getBoundaryPoint(0).m_x < _boundary->getBoundaryPoint(2).m_x)?_boundary->getBoundaryPoint(0).m_x:_boundary->getBoundaryPoint(2).m_x;
  startX += (0.5 * m_width);
  startX /= m_cellSize;
  startX = (startX >= 0)?startX:0;

  int endX = (int)(_boundary->getBoundaryPoint(0).m_x > _boundary->getBoundaryPoint(2).m_x)?_boundary->getBoundaryPoint(0).m_x:_boundary->getBoundaryPoint(2).m_x;
  endX += (0.5 * m_width);
  endX /= m_cellSize;
  endX = (endX <= m_numXcells)?endX:m_numXcells;

  int startY = (int)(_boundary->getBoundaryPoint(0).m_z < _boundary->getBoundaryPoint(2).m_z)?_boundary->getBoundaryPoint(0).m_z:_boundary->getBoundaryPoint(2).m_z;
  startY += (0.5 * m_width);
  startY /= m_cellSize;
  startY = (startY >= 0)?startY:0;

  int endY = (int)(_boundary->getBoundaryPoint(0).m_z > _boundary->getBoundaryPoint(2).m_z)?_boundary->getBoundaryPoint(0).m_z:_boundary->getBoundaryPoint(2).m_z;
  endY += (0.5 * m_width);
  endY /= m_cellSize;
  endY = (endY <= m_numYcells)?endY:m_numYcells;

  for(int i=startX-1; i<=endX+1; i++)
  {
      for(int j=startY-1; j<=endY+1; j++)
      {
          // add this boundary to cells with coord (i,j)
          //m_cells[(j * m_numXcells) + i].m_bounds.push_back(_boundary);
          if(i>=0 && i<= m_numXcells && j >=0 && j <= m_numYcells)
          {
              getCell(i,j)->m_bounds.push_back(_boundary);
              _boundary->setHashID((j*m_numXcells)+i);
          }
          // add boundary to agents in this cell
          // make this part another function
          // because it can then be called seperate from addbound
      }
  }
}

void HashTable::addBoundary4ToHash(Boundary4* _boundary)
{
}

/*  =======not working do not use!=======
void HashTable::addBoundaryToAgent(Boundary* _boundary)
{
  std::vector<int> cells = _boundary->getHashID();
  std::cout<<"cell contains: "<<cells.size()<<" num bounds\n";
    for(unsigned int i=0;i<_boundary->getHashID().size();i++)
    {
        // iterate through number of cells boundary spans

        // all agent in cell[_boundary->getHashID()[i]] add boundary to once!
        // must check if boundary is already in agent

        for(unsigned int j=0;j<m_cells[_boundary->getHashID()[i]].m_agents.size();j++)
        {
            //iterate through number of agents in cell
            // boundary to agent in this cell
            m_cells[_boundary->getHashID()[i]].m_agents[j]->getBrain()->addBoundary(_boundary);
            std::vector<Boundary*>::iterator it;
            it = std::find(m_cells[_boundary->getHashID()[i]].m_agents[j]->getBrain()->getBoundaries().begin(),
                           m_cells[_boundary->getHashID()[i]].m_agents[j]->getBrain()->getBoundaries().end(),
                           _boundary);
        }
    }
}
*/

void HashTable::addBoundaryToAgent()
{
    for(unsigned int i=0;i<m_cells.size();i++)
    {
        // iterate hrough all cells and if they have a boundary and agent add them
        if(m_cells[i].m_agents.size() != 0)
        {
            if(m_cells[i].m_bounds.size() != 0)
            {
                //add boudaries to agents
                for(unsigned int j=0;j<m_cells[i].m_agents.size();j++)
                {
                    for(unsigned int k=0;k<m_cells[i].m_bounds.size();k++)
                    {
                        std::vector<Boundary*>::iterator it;
                        it = std::find(m_cells[i].m_agents[j]->getBrain()->getBoundaries().begin(),
                                       m_cells[i].m_agents[j]->getBrain()->getBoundaries().end(),
                                       m_cells[i].m_bounds[k]);
                        if(it == m_cells[i].m_agents[j]->getBrain()->getBoundaries().end())
                        {
                            // boundary not found
                            m_cells[i].m_agents[j]->getBrain()->addBoundary(m_cells[i].m_bounds[k]);
                        }
                    }
                }
            }
        }
    }
}

Cell *HashTable::getCell(int _x, int _y)
{
    // make sure position is within hash table
    if(_x < 0)           {_x=0;}
    if(_x >= m_numXcells){_x=m_numXcells-1;}
    if(_y <0 )           {_y=0;}
    if(_y >= m_numYcells){_y=m_numYcells-1;}

    return &(m_cells[(_y * m_numXcells) + _x]);
}

Cell *HashTable::getCell(const ngl::Vec3 &_pos)
{
    // work out the index value to look into table from agents pos
    // must shift by half width and half height because hash table starts at the origin, not centred at origin
    int x,y;
    x = (int)(_pos.m_x + 0.5 * m_width) / m_cellSize;
    y = (int)(_pos.m_z + 0.5 * m_height) / m_cellSize;

    // return cell
    return getCell(x,y);
}

Cell *HashTable::getCell(const Agent &_agent)
{
    return getCell(_agent.getOrigState().m_pos);
}

std::vector<Cell> HashTable::getCells()const
{
    return m_cells;
}

void HashTable::addNeighbours()
{
    // loop through all cells
    for(unsigned int i=0;i<m_cells.size();i++)
    {
        int x = i % m_numXcells;
        int y = (int) (i / m_numXcells);

        // loops through all agents in cell
        for(unsigned int j=0;j<m_cells[i].m_agents.size();j++)
        {
            // check whether agent is in perceived radius of other agents in cell
            checkNeighboursInCell(m_cells[i].m_agents[j],m_cells[i].m_agents,j+1);

            // check neighbouring cells
            if(x>0)
            {
                //check left hand side cells
                checkNeighboursInCell(m_cells[i].m_agents[j],getCell(x-1,y)->m_agents,0);
                //checkNeighboursInCell(m_cells[i].m_agents[j],m_cells[i - 1].m_agents,0);
            }
            if(y>0)
            {
                // cehck bottom cell
                checkNeighboursInCell(m_cells[i].m_agents[j],getCell(x,y-1)->m_agents,0);
            }
            if(x>0 && y< m_numYcells)
            {
                // check top left cell
                checkNeighboursInCell(m_cells[i].m_agents[j],getCell(x-1,y+1)->m_agents,0);
            }
            if(x>0 && y>0)
            {
                // check bottom left cell
                checkNeighboursInCell(m_cells[i].m_agents[j],getCell(x-1,y-1)->m_agents,0);
            }
        }

    }
}

void HashTable::checkNeighboursInCell(Agent *currentAgent,std::vector<Agent*>_testAgents,int startIndex)
{
    // iterate through agents in cell
    for(unsigned int i=startIndex;i<_testAgents.size();i++)
    {
        // work out distance between agents
        ngl::Vec3 distV = currentAgent->getCurrentState().m_pos - _testAgents[i]->getCurrentState().m_pos;
        float distL = distV.length();
        distL -= (currentAgent->getCurrentState().m_rad + _testAgents[i]->getCurrentState().m_rad);

        // if distance+radius is less than perceived radius add to neighbour list
        if(distL <= currentAgent->getBrain()->getPerceiveRad())
        {
            // add to each others neighbours
            currentAgent->getBrain()->addNeighbour(_testAgents[i]);
            _testAgents[i]->getBrain()->addNeighbour(currentAgent);
        }
    }

}

void HashTable::initVAO()
{
  if(m_isVAOinit){return;}
    std::vector<ngl::Vec3> verts;

    for(int i=0;i<m_numXcells+1;i++)
    {
        ngl::Vec3 p1,p2;
        p1 = ngl::Vec3((i*m_cellSize) - (0.5*m_width),0,0.5*m_height);
        p2 = ngl::Vec3((i*m_cellSize) - (0.5*m_width),0,-0.5*m_height);
        verts.push_back(p1);
        verts.push_back(p2);
    }

    for(int i=0;i<m_numYcells+1;i++)
    {
        ngl::Vec3 p1,p2;
        p1 = ngl::Vec3(0.5*m_width,0,(i*m_cellSize) - (0.5*m_height));
        p2 = ngl::Vec3(-0.5*m_width,0,(i*m_cellSize) - (0.5*m_height));
        verts.push_back(p1);
        verts.push_back(p2);
    }

    m_vao = ngl::VertexArrayObject::createVOA(GL_LINES);
    m_vao->bind();
    m_vao->setData(verts.size()*sizeof(ngl::Vec3),verts[0].m_x);
    m_vao->setVertexAttributePointer(0,3,GL_FLOAT,0,0);
    m_vao->setNumIndices(verts.size());
    m_vao->unbind();

    m_cellVao = ngl::VertexArrayObject::createVOA(GL_TRIANGLE_STRIP);

    m_isVAOinit = true;
}
void HashTable::draw()
{
  if(!m_isVAOinit){return;}
  std::vector<ngl::Vec3> verts;

  for(int i=0;i<m_numXcells+1;i++)
  {
      ngl::Vec3 p1,p2;
      p1 = ngl::Vec3((i*m_cellSize) - (0.5*m_width),0,0.5*m_height);
      p2 = ngl::Vec3((i*m_cellSize) - (0.5*m_width),0,-0.5*m_height);
      verts.push_back(p1);
      verts.push_back(p2);
  }

  for(int i=0;i<m_numYcells+1;i++)
  {
      ngl::Vec3 p1,p2;
      p1 = ngl::Vec3(0.5*m_width,0,(i*m_cellSize) - (0.5*m_height));
      p2 = ngl::Vec3(-0.5*m_width,0,(i*m_cellSize) - (0.5*m_height));
      verts.push_back(p1);
      verts.push_back(p2);
  }

  m_vao->bind();
  m_vao->updateData(verts.size()*sizeof(ngl::Vec3),verts[0].m_x);
  m_vao->setVertexAttributePointer(0,3,GL_FLOAT,0,0);
  m_vao->draw();
  m_vao->unbind();
}

void HashTable::printInfo()const
{
    int x,y,numCells,numAgents;
    numCells = m_numXcells * m_numYcells;
    y = -1;
    x = 0;
    numAgents = 0;
    for(int i=0;i<numCells;i++)
    {
        if(x == 0)
        {
            y++;
        }
        numAgents += m_cells[(y*m_numXcells) + x].m_agents.size();
        std::cout<<"x: "<<x<<" y: "<<y<<" numAgents in cell: "<<m_cells[(y*m_numXcells) + x].m_agents.size()<<"\n";
        x++;
        x = x % m_numXcells;
    }
    std::cout<<"total agents: "<<numAgents<<"\n---------------------------------------\n";
}
