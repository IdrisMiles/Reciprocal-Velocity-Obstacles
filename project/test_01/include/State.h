#ifndef _STATE__H_
#define _STATE__H_

#include <ngl/Vec3.h>


class State
{
public:
    State()
    {
        m_pos     = ngl::Vec3(0.0f,0.0f,0.0f);
        m_vel     = ngl::Vec3(0.0f,0.0f,0.0f);
        m_acc     = ngl::Vec3(0.0f,0.0f,0.0f);
        m_force   = ngl::Vec3(0.0f,0.0f,0.0f);
        m_mass    = 1.0f;
        m_invMass = 1.0f;
        m_orien   = 0.0f;
        m_rad     = 0.01f;
    }

    State(const State &_state)
    {
        m_pos     = _state.m_pos;
        m_vel     = _state.m_vel;
        m_acc     = _state.m_acc;
        m_force   = _state.m_force;
        m_mass    = _state.m_mass;
        m_invMass = _state.m_invMass;
        m_orien   = _state.m_orien;
        m_rad     = _state.m_rad;
    }

    ~State(){}

    ngl::Vec3 m_pos;
    ngl::Vec3 m_vel;
    ngl::Vec3 m_acc;
    ngl::Vec3 m_force;
    float m_mass;
    float m_invMass;
    float m_orien;
    float m_rad;


private:

};


#endif //_STATE__H_
