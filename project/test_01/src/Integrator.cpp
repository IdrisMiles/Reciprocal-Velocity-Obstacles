#include "Integrator.h"

Integrator::Integrator()
{

}

Integrator::~Integrator()
{

}

void Integrator::update()
{
    switch (m_type)
    {
        case RK4:
            integrateRK4(0.1f); break;
        case EULER:
            integrateEuler(0.1f); break;
        default:
            break;
    }
}

void Integrator::setType(const integratorType &_type)
{
    m_type = _type;
}

void Integrator::setState(State *_state)
{
    m_state = _state;
}

State *Integrator::getState()const
{
    return m_state;
}

//--------RK4 integrating methods-----------
ngl::Vec3 Integrator::motion(const State &_state)
{
    return (m_state->m_invMass * m_state->m_force);
}

State Integrator::evaluate(const float _dt,  const State &_deriv)
{
    State state;
    state.m_pos = (m_state->m_pos + (_deriv.m_pos * _dt));
    state.m_vel = (m_state->m_vel + (_deriv.m_vel * _dt));

    State output;
    output.m_pos = (m_state->m_vel);
    output.m_vel = (motion(*m_state));

    return output;
}

State Integrator::evaluate()
{
    State output;
    output.m_pos = (m_state->m_vel);
    output.m_vel = (motion(*m_state));
    return output;
}

void Integrator::integrateRK4(const float &_dt)
{
    State a = evaluate();
    State b = evaluate(_dt * 0.5, a);
    State c = evaluate(_dt * 0.5, b);
    State d = evaluate(_dt,       c);

    ngl::Vec3 dxdt = (1.0 / 6.0) * (a.m_pos + 2 * (b.m_pos + c.m_pos) + d.m_pos);
    ngl::Vec3 dvdt = (1.0 / 6.0) * (a.m_vel + 2 * (b.m_vel + c.m_vel) + d.m_vel);

    m_state->m_pos = (m_state->m_pos + (dxdt * _dt));
    m_state->m_vel = (m_state->m_vel + (dvdt * _dt));
}

//--------Euler integrating methods---------
void Integrator::integrateEuler(const float &_dt)
{
    m_state->m_acc = m_state->m_invMass * m_state->m_force;
    m_state->m_vel = m_state->m_acc * _dt;
    m_state->m_pos += m_state->m_vel * _dt;
}
