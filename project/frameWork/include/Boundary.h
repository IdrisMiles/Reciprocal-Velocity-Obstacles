#ifndef _BOUNDARY__H_
#define _BOUNDARY__H_

#include <vector>
#include <ngl/Vec3.h>

struct Cell;

class Boundary
{
public:

    Boundary();
    Boundary(const ngl::Vec3 &_p1, const ngl::Vec3 &_p2);
    ~Boundary();

    void setBoundary(const ngl::Vec3 &_p1, const ngl::Vec3 &_p2);
    void setBoundary(const int &_i,const ngl::Vec3 &_p);

    ngl::Vec3 *getBoundaryPoints();
    ngl::Vec3 getBoundaryPoint(const int &_i)const;

    // Hash table stuff
    void setHashID(const int &_id);
    void setCellID(const int &_id);
    void setCell(Cell *_cell);

    Cell* getCell();
    int getCellID();
    std::vector<int> getHashID()const;

private:

        ngl::Vec3 m_p[2];

        std::vector<int> m_hashTableID;
        int m_cellID;
        Cell *m_cell;
};



#endif //_BOUNDARY__H_
