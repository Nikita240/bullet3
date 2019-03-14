#ifndef GROUND_BODY_H
#define GROUND_BODY_H

#include "btBulletDynamicsCommon.h"

#include "RigidBody.h"
#include "GroundShape.h"

class GroundBody : public RigidBody
{
		public:
				GroundBody(GroundShape* a_collisionShape) : RigidBody(a_collisionShape, 0, btVector3(0, -a_collisionShape->getHalfExtentsWithMargin().getY(), 0)) {};
};

#endif