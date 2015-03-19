#ifndef _AGENT__H_
#define _AGENT__H_


#include<boost/shared_ptr.hpp>

#include <ngl/Obj.h>
#include <ngl/Camera.h>
#include <ngl/Transformation.h>
#include <ngl/Mat4.h>
#include <ngl/Material.h>
#include <ngl/BBox.h>

#include "State.h"
#include "Brain.h"
#include "System.h"
#include "Integrator.h"



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

    ngl::BBox m_bbox;
    ngl::Obj *m_mesh;
    ngl::Material m_material;

    System *m_system;
    Brain *m_brain;
    Integrator m_integrator;

    int m_hashTableID;
    int m_cellID;

};





#endif //_AGENT__H_
