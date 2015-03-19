#ifndef _HASH__H_
#define _HASH__H_

#include <vector>
#include<boost/shared_ptr.hpp>
#include <ngl/Vec3.h>
#include <ngl/BBox.h>

#include "Agent.h"


class Cell
{
public:
    std::vector<Agent*> m_agents;
    int m_hashTableID;
};

class HashTable
{

public:
    HashTable(int _width, int _height, int _cellSize,
              const ngl::Vec3 &_centre = ngl::Vec3(0.0f,0.0f,0.0f));
    HashTable(const ngl::BBox &_bound, int _cellSize);
    ~HashTable();

    void addAgent(Agent* _agent);
    void removeAgent(Agent* _agent);

    Cell *getCell(int _x, int _y);
    Cell *getCell(const ngl::Vec3 &_pos);
    Cell *getCell(const Agent &_agent);

private:

    std::vector<Cell*> m_cells;
    int m_cellSize;
    int m_width;
    int m_height;
    int m_numXcells;
    int m_numYcells;
    ngl::Vec3 m_centre; //real world centre

};


#endif // _HASH__H_
