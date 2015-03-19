#ifndef _BOUNDARY__H_
#define _BOUNDARY__H_

#include <vector>
#include <ngl/Vec3.h>

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

private:

        ngl::Vec3 m_p[2];
};



#endif //_BOUNDARY__H_
