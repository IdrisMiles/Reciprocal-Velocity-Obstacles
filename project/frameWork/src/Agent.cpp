#include "Agent.h"

#include <ngl/VAOPrimitives.h>
#include <ngl/ShaderLib.h>
#include <ngl/Random.h>
#include <ngl/NGLStream.h>


Agent::Agent(System *_system, const Avoidance &_avoidType)
{
    ngl::Random *r = ngl::Random::instance();
    m_currentState.m_pos = 7.0f * r->getRandomVec3();
    m_origState = m_currentState;

    // set up AABB - will be used for spatial hashing
    m_bbox = ngl::BBox(m_currentState.m_pos,m_currentState.m_rad,
                       m_currentState.m_rad,m_currentState.m_rad);


    // linking system to agent
    m_system = _system;

    // initializing brain
    m_brain = new Brain(this,_system);
    m_brain->setAvoidanceType(_avoidType);

    // setting up integrator
    m_integrator.setType(EULER);
    m_integrator.setDesSpeed(m_brain->getDesSpeed());
    m_integrator.setState(&m_currentState);

    // hash table stuff
    m_cellID = -1;


    // TODO: set up obj mesh

    // Setting up avoidant specific attributes
    switch (_avoidType)
    {
      case FLOCKING:
        {
          m_material.set(ngl::BRONZE);
          break;
        }

      case RVO:
        {m_material.set(ngl::GOLD);
        break;}

      case SOCIAL:
        {m_material.set(ngl::SILVER);
        break;}

      default:
        break;
    }

}

Agent::~Agent()
{
    delete [] m_brain;
}

//-------------update stuff-----------------
void Agent::update()
{
    // update brain, find new path/forces/velocity etc
    m_brain->update();
    // update position and move agent
    m_integrator.update();
}

void Agent::updateState()
{
    m_origState = m_currentState;
}

//-------------drawing stuff-------------
void Agent::draw()
{
    ngl::VAOPrimitives *prim = ngl::VAOPrimitives::instance();
    prim->draw("cylinder");

}

void Agent::loadMatricesToShader()
{
  ngl::ShaderLib *shader=ngl::ShaderLib::instance();
  m_material.loadToShader("material");

  ngl::Mat4 MV;
  ngl::Mat4 MVP;
  ngl::Mat3 normalMatrix;
  ngl::Mat4 worldM;
  ngl::Mat4 M;
  M.identity();
  ngl::Mat4 stand;
  stand.rotateX(90);
  // add agents orientation
  M.rotateY(m_currentState.m_orien);
  // add agents position
  M.m_m[3][0] = m_currentState.m_pos.m_x;
  //M.m_m[3][1] = m_currentState.m_pos.m_y;
  M.m_m[3][2] = m_currentState.m_pos.m_z;

  worldM   = stand * M * m_system->getGlobalTX();
  MV  = worldM * m_system->getCam().getViewMatrix();
  MVP = worldM * m_system->getCam().getVPMatrix();
  normalMatrix = MV;
  normalMatrix.inverse();

  shader->setShaderParamFromMat4("MV",MV);
  shader->setShaderParamFromMat4("MVP",MVP);
  shader->setShaderParamFromMat3("normalMatrix",normalMatrix);
  shader->setShaderParamFromMat4("M",worldM);
}

//------------getters-----------------
Brain *Agent::getBrain()
{
    return m_brain;
}

State Agent::getCurrentState()const
{
    return m_currentState;
}
State Agent::getOrigState()const
{
    return m_origState;
}

//-------------setters------------------
void Agent::setState(const State &_state)
{
    m_currentState = _state;
}

void Agent::setForce(const ngl::Vec3 &_force)
{
    m_currentState.m_force = _force;
}

void Agent::setVel(const ngl::Vec3 &_vel)
{
    m_currentState.m_vel = _vel;
}

void Agent::setPos(const ngl::Vec3 &_pos)
{
    m_currentState.m_pos = _pos;
}

void Agent::setGoal(const ngl::Vec3 &_goal)
{
  m_brain->setGoal(_goal);
}


void Agent::setHashID(const int &_id)
{
    m_hashTableID = _id;
}

void Agent::setCellID(const int &_id)
{
    m_cellID = _id;
}

void Agent::setCell(Cell *_cell)
{
    m_cell = _cell;
}


Cell *Agent::getCell()
{
    return m_cell;
}

int Agent::getCellID()
{
    return m_cellID;
}


void Agent::printInfo()const
{
  std::cout<<"Agent info:\n";
  std::cout<<"Cell ID: "<<m_cellID<<"\n";
  std::cout<<"orig pos: "<<m_origState.m_pos<<"\n";
  std::cout<<"orig vel: "<<m_origState.m_vel<<"\n";
  m_brain->printInfo();
  std::cout<<"-------------------------------\n\n";
}
