#include "Boundary.h"
#include <ngl/ShaderLib.h>

Boundary::Boundary()
{

}

Boundary::Boundary(const ngl::Vec3 &_p1, const ngl::Vec3 &_p2, const bool &_drawFlag)
{
    m_p[0] = _p1;
    m_p[1] = _p2;
    m_isVAOinit = false;

    if(_drawFlag){initVAO();}
}

Boundary::~Boundary()
{

  m_vao->removeVOA();
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
//=========drawing stuff===========
void Boundary::initVAO()
{
  if(m_isVAOinit){return;}

  std::vector<ngl::Vec3> verts;
  verts.push_back(m_p[0]);
  verts.push_back(m_p[0] + ngl::Vec3(0,2,0));
  verts.push_back(m_p[1]);
  verts.push_back(m_p[1] + ngl::Vec3(0,2,0));

  m_vao = ngl::VertexArrayObject::createVOA(GL_TRIANGLE_STRIP);
  m_vao->bind();
  m_vao->setData(verts.size()*sizeof(ngl::Vec3),verts[0].m_x);
  m_vao->setVertexAttributePointer(0,3,GL_FLOAT,0,0);
  m_vao->setNumIndices(verts.size());
  m_vao->unbind();

  m_isVAOinit = true;
}

void Boundary::loadMatricesToShader(ngl::Mat4 &_globalTX,ngl::Camera &_cam)
{
  ngl::ShaderLib *shader = ngl::ShaderLib::instance();
  shader->use("Wall");

  ngl::Mat4 MV;
  ngl::Mat4 MVP;
  ngl::Mat3 normalMatrix;
  ngl::Mat4 M;

  M   = _globalTX;
  MV  = M*_cam.getViewMatrix();
  MVP = M*_cam.getVPMatrix();
  normalMatrix=MV;
  normalMatrix.inverse();

  shader->setShaderParamFromMat4("MVP",MVP);

}

void Boundary::updateVAO()
{
  if(!m_isVAOinit){return;}

}

void Boundary::draw()
{
  if(!m_isVAOinit){return;}

  std::vector<ngl::Vec3> verts;
  verts.push_back(m_p[0]);
  verts.push_back(m_p[0] + ngl::Vec3(0,2,0));
  verts.push_back(m_p[1]);
  verts.push_back(m_p[1] + ngl::Vec3(0,2,0));

  m_vao->bind();
  m_vao->updateData(verts.size()*sizeof(ngl::Vec3),verts[0].m_x);
  m_vao->setVertexAttributePointer(0,3,GL_FLOAT,0,0);
  m_vao->draw();
  m_vao->unbind();

}


//=======hash table stuff==========
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
