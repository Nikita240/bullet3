#ifndef GROUND_SHAPE_H
#define GROUND_SHAPE_H

#include "btBulletDynamicsCommon.h"

#include "BoxShape.h"

class GroundShape : public BoxShape
{
	public:
		GroundShape(const btVector3& a_boxHalfExtents = btVector3(10, 0.5, 10)) : BoxShape(a_boxHalfExtents) {};
};

#endif