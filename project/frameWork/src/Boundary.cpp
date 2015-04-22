#include "Boundary.h"

Boundary::Boundary()
{

}

Boundary::Boundary(const ngl::Vec3 &_p1, const ngl::Vec3 &_p2)
{
    m_p[0] = _p1;
    m_p[1] = _p2;
}

Boundary::~Boundary()
{

}

void Boundary::setBoundary(const int &_i, const ngl::Vec3 &_p)
{
    m_p[_i] = _p;
}

void Boundary::setBoundary(const ngl::Vec3 &_p1, const ngl::Vec3 &_p2)
{
    m_p[0] = _p1;
    m_p[1] = _p2;
}

ngl::Vec3 *Boundary::getBoundaryPoints()
{
    return m_p;
}

ngl::Vec3 Boundary::getBoundaryPoint(const int &_i) const
{
    return m_p[_i];
}

// hash table stuff


void Boundary::setHashID(const int &_id)
{
    m_hashTableID.push_back(_id);
}

void Boundary::setCellID(const int &_id)
{
    m_cellID = _id;
}

void Boundary::setCell(Cell *_cell)
{
    m_cell = _cell;
}


Cell *Boundary::getCell()
{
    return m_cell;
}

int Boundary::getCellID()
{
    return m_cellID;
}

std::vector<int> Boundary::getHashID()const
{
    return m_hashTableID;
}
