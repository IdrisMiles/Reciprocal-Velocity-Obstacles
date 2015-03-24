#include "HashTable.h"

HashTable::HashTable(int _width, int _height, int _cellSize, const ngl::Vec3 &_centre)
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
    Cell *tmpCell = getCell(*_agent);
    tmpCell->m_agents.push_back(_agent);
}

void HashTable::removeAgent(Agent* _agent)
{

}


Cell *HashTable::getCell(int _x, int _y)
{
    if(_x < 0)           {_x=0;}
    if(_x >= m_numXcells){_x=m_numXcells-1;}
    if(_y <0 )           {_y=0;}
    if(_y >= m_numYcells){_y=m_numYcells-1;}

    return m_cells[(_y * m_numXcells) + _x];
}

Cell *HashTable::getCell(const ngl::Vec3 &_pos)
{
    // work out the index value to look into table from agents pos
    int x,y;
    x = (int)(_pos.m_x / m_cellSize);
    y = (int)(_pos.m_z / m_cellSize);

    // return cell
    return getCell(x,y);
}

Cell *HashTable::getCell(const Agent &_agent)
{
    return getCell(_agent.getOrigState().m_pos);
}
