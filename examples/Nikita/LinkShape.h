#ifndef LINK_SHAPE_H
#define LINK_SHAPE_H

#include "btBulletDynamicsCommon.h"

class LinkShape : public btBU_Simplex1to4
{
public:
	LinkShape(const btScalar length = 1.) : btBU_Simplex1to4(btVector3(length, 0, 0), btVector3(0, 0, 0)){};

	LinkShape(const btVector3 pointA, const btVector3 pointB) : btBU_Simplex1to4(pointA, pointB){};
};

#endif