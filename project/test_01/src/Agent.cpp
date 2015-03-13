#include "Agent.h"

#include <ngl/VAOPrimitives.h>
#include <ngl/ShaderLib.h>
#include <ngl/Random.h>

Agent::Agent(System *_system, const Avoidance &_avoidType)
{
    // ctor that should be used

    ngl::Random *r = ngl::Random::instance();
    m_currentState.m_pos = r->getRandomVec3();
    m_currentState.m_pos *= 5.0f;

    m_origState = m_currentState;

    m_rad = 0.1f;

    // linking system to agent
    m_system = _system;
    // initializing brain
    m_brain = new Brain(this,_system);
    m_brain->setAvoidanceType(_avoidType);

    // setting up integrator
    m_integrator.setType(EULER);
    m_integrator.setState(&m_currentState);

    ngl::VAOPrimitives *prim = ngl::VAOPrimitives::instance();
    prim->createCylinder("cylinder",m_currentState.m_rad,1.0,20,20);
}

Agent::~Agent()
{
    delete [] m_brain;
}

void Agent::update()
{
    m_brain->update();
    m_integrator.update();
    //m_currentState.m_pos.m_x += 0.01f;
    //m_currentState.m_orien += 0.5f;
}

void Agent::updateState()
{
    m_origState = m_currentState;
}

void Agent::draw()
{
    ngl::VAOPrimitives *prim = ngl::VAOPrimitives::instance();
    prim->draw("troll");
}

void Agent::loadMatricesToShader()
{
  ngl::ShaderLib *shader=ngl::ShaderLib::instance();

  ngl::Mat4 MV;
  ngl::Mat4 MVP;
  ngl::Mat3 normalMatrix;
  ngl::Mat4 worldM;
  ngl::Mat4 M;
  M.identity();
  ngl::Mat4 stand;
  //stand.rotateX(90);
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

float Agent::getRad()const
{
    return m_rad;
}


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
