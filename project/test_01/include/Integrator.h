#ifndef _INTEGRATOR__H_
#define _INTEGRATOR__H_

#include"State.h"

enum integratorType {RK4,EULER};

class Integrator
{
public:
    Integrator();
    ~Integrator();

    void update();

    void setType(const integratorType &_type);
    void setState(State *_state);
    void setDesSpeed(const float &_desSpeed);
    State *getState()const;

    //--------RK4 integrating methods-----------
    ngl::Vec3 motion(const State &_state);
    State evaluate(const float _dt, const State &_deriv);
    State evaluate();
    void integrateRK4(const float &_dt);

    //--------Euler integrating methods---------
    void integrateEuler(const float &_dt);

private:

    State *m_state;
    State m_deriv;
    float m_desSpeed;

    integratorType m_type;

};

#endif //_INTEGRATOR__H_
