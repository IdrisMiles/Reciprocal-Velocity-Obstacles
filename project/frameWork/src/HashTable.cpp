#include "HashTable.h"
#include <iostream>

HashTable::HashTable(int _width, int _height, float _cellSize, const ngl::Vec3 &_centre)
{
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

}

HashTable::~HashTable()
{

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
    //_agent->getCell()->m_agents[_agent->getCellID()] = _agent->getCell()->m_agents.back();
    std::swap(_agent->getCell()->m_agents[_agent->getCellID()],_agent->getCell()->m_agents.back());

    // push last element (this agent) off
    _agent->getCell()->m_agents.pop_back();

    // if the agent was the only agent in the cell
    if(_agent->getCell()->m_agents.size() != 0)
    {
        // update swapped agents cellID
        _agent->getCell()->m_agents[_agent->getCellID()]->setCellID(_agent->getCellID());
    }

    // update current agents cellID
    _agent->setCellID(-1);

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
    int x,y;
    x = (int)(_pos.m_x / m_cellSize) + (int)(0.5 * m_width);
    y = (int)(_pos.m_z / m_cellSize) + (int)(0.5 * m_height);

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
    for(int i=0;i<m_cells.size();i++)
    {
        int x = i % m_numXcells;
        int y = (int) (i / m_numXcells);

        // loops through all agents in cell
        for(int j=0;j<m_cells[i].m_agents.size();j++)
        {
            // check whether agent is in perceived radius of other agents in cell
            checkCollisionOnCell(m_cells[i].m_agents[j],m_cells[i].m_agents,j+1);

            // check neighbouring cells
//            if(x>0)
//            {
//                //check left hand side cells
//                checkCollisionOnCell(m_cells[i].m_agents[j],m_cells[].m_agents,0);
//                checkCollisionOnCell(m_cells[i].m_agents[j],m_cells[].m_agents,0);
//                checkCollisionOnCell(m_cells[i].m_agents[j],m_cells[].m_agents,0);
//            }
//            if(y>0)
//            {
//                // cehck bottom cell
//                checkCollisionOnCell(m_cells[i].m_agents[j],m_cells[].m_agents,0);
//            }
        }

    }
}

void HashTable::checkCollisionOnCell(Agent *currentAgent,std::vector<Agent*>_testAgents,int startIndex)
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



void HashTable::printInfo()const
{
    int x,y,numCells,numAgents;
    numCells = m_numXcells * m_numYcells;
    y = -1;
    x = 0;
    numAgents = 0;
    for(unsigned int i=0;i<numCells;i++)
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
