#ifndef BOX_SHAPE_H
#define BOX_SHAPE_H

#include "btBulletDynamicsCommon.h"

class BoxShape : public btBoxShape
{
    public:
        BoxShape(const btVector3& a_boxHalfExtents = btVector3(.1, .1, .1)) : btBoxShape(a_boxHalfExtents) {};
};

#endif