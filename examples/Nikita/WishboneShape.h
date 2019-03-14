#ifndef WISHBONE_SHAPE_H
#define WISHBONE_SHAPE_H

#include "btBulletDynamicsCommon.h"

class WishboneShape : public btCompoundShape
{
    public:
        WishboneShape(
            const btVector3 a_knucklePoint, 
            const btVector3 a_subframePointA, 
            const btVector3 a_subframePointB
        );
};

#endif