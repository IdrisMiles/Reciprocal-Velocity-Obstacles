#ifndef _AGENT__H_
#define _AGENT__H_


#include<boost/shared_ptr.hpp>

#include <ngl/Camera.h>
#include <ngl/Transformation.h>
#include <ngl/Mat4.h>
#include <ngl/Material.h>
#include "State.h"
#include "Brain.h"
#include "Integrator.h"
#include "System.h"


class State;
class Brain;
class System;


class Agent
{
public:

    Agent(System *_system, const Avoidance &_avoidType);
    ~Agent();

    void update();
    void updateState();
    void draw();
    void loadMatricesToShader();

    Brain *getBrain();
    State getCurrentState()const;
    State getOrigState()const;
    float getRad()const;

    void setState(const State &_state);
    void setForce(const ngl::Vec3 &_force);
    void setVel(const ngl::Vec3 &_vel);
    void setPos(const ngl::Vec3 &_pos);

private:

    State m_currentState;
    State m_origState;
    System *m_system;
    Brain *m_brain;
    Integrator m_integrator;
    float m_rad;
    ngl::Material m_material;

};





#endif //_AGENT__H_
