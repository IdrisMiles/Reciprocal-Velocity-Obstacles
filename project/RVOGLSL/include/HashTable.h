#ifndef _HASH__H_
#define _HASH__H_

#include <vector>
#include<boost/shared_ptr.hpp>
#include <ngl/Vec3.h>
#include <ngl/BBox.h>
#include <ngl/VertexArrayObject.h>
#include <ngl/VAOPrimitives.h>

#include "Agent.h"
#include "boundary4.h"


struct Cell
{
    std::vector<Agent*> m_agents;
    std::vector<Boundary*> m_bounds;
    std::vector<Boundary4*> m_bounds4;
};

class HashTable
{

public:
    HashTable(int _width, int _height, float _cellSize,
              const ngl::Vec3 &_centre = ngl::Vec3(0.0f,0.0f,0.0f));
    HashTable(const ngl::BBox &_bound, int _cellSize);
    ~HashTable();

    void addAgent(Agent* _agent);
    void addAgent(Agent* _agent, Cell* _cell);
    void removeAgent(Agent* _agent);
    void emptyAgents();
    void emptyTable();

    void addBoundaryToHash(Boundary* _boundary);
    void addBoundary4ToHash(Boundary4* _boundary);
    void addBoundaryToAgent(Boundary* _boundary);
    void addBoundaryToAgent();

    Cell *getCell(int _x, int _y);
    Cell *getCell(const ngl::Vec3 &_pos);
    Cell *getCell(const Agent &_agent);
    std::vector<Cell> getCells()const;

    void addNeighbours();
    void checkNeighboursInCell(Agent *currentAgent, std::vector<Agent *> _testAgents, int startIndex);

    void initVAO();
    void updateVAO();
    void draw();

    void printInfo()const;

private:

    std::vector<Cell> m_cells;
    float m_cellSize;
    int m_width;
    int m_height;
    int m_numXcells;
    int m_numYcells;
    ngl::Vec3 m_centre; //real world centre


    bool m_isVAOinit;
    ngl::VertexArrayObject *m_vao;
    ngl::VertexArrayObject *m_cellVao;

};


#endif // _HASH__H_
