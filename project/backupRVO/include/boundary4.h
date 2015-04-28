#ifndef _BOUNDARY4__H_
#define _BOUNDARY4__H_

#include "Boundary.h"
#include <vector>

class Boundary4
{
public:
    Boundary4(Boundary* _bound0, Boundary* _bound1,
              Boundary* _bound2, Boundary* _bound3);
    ~Boundary4();

    std::vector<Boundary *> getBounds()const;

private:
    std::vector<Boundary*> m_bounds;
};

#endif //_BOUNDARY4__H_
