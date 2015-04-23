#ifndef _BOUNDARY__H_
#define _BOUNDARY__H_

#include <vector>
#include <ngl/Vec3.h>
#include <ngl/VertexArrayObject.h>
#include <ngl/Mat4.h>
#include <ngl/Camera.h>

struct Cell;

class Boundary
{
public:

    Boundary();
    Boundary(const ngl::Vec3 &_p1, const ngl::Vec3 &_p2, const bool &_drawFlag = false);
    ~Boundary();

    void setBoundary(const ngl::Vec3 &_p1, const ngl::Vec3 &_p2);
    void setBoundary(const int &_i,const ngl::Vec3 &_p);

    ngl::Vec3 *getBoundaryPoints();
    ngl::Vec3 getBoundaryPoint(const int &_i)const;

    // drawing stuff
    void loadMatricesToShader(ngl::Mat4 &_globalTX, ngl::Camera &_cam);
    void initVAO();
    void updateVAO();
    void draw();

    // Hash table stuff
    void setHashID(const int &_id);
    void setCellID(const int &_id);
    void setCell(Cell *_cell);

    Cell* getCell();
    int getCellID();
    std::vector<int> getHashID()const;

private:

        ngl::Vec3 m_p[2];

        bool m_isVAOinit;
        ngl::VertexArrayObject *m_vao;

        std::vector<int> m_hashTableID;
        int m_cellID;
        Cell *m_cell;
};



#endif //_BOUNDARY__H_
