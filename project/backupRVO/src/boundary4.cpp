#include "boundary4.h"

Boundary4::Boundary4(Boundary *_bound0, Boundary *_bound1, Boundary *_bound2, Boundary *_bound3)
{

}

Boundary4::~Boundary4()
{

}

std::vector<Boundary*> Boundary4::getBounds()const
{
    return m_bounds;
}
