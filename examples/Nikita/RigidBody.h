#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include "btBulletDynamicsCommon.h"

class RigidBody : public btRigidBody
{
	public:
		RigidBody(
			btCollisionShape* a_collisionShape,
			btScalar a_mass = 0,
			btTransform a_startTransform = btTransform::getIdentity()) : btRigidBody(RigidBody::defaultConstructionInfo(a_collisionShape, a_mass, new btDefaultMotionState(a_startTransform))) {};

		RigidBody(
			btCollisionShape* a_collisionShape,
			btScalar a_mass,
			btVector3 a_startPosition,
			btMatrix3x3 a_startRotation = btMatrix3x3::getIdentity()) : btRigidBody(RigidBody::defaultConstructionInfo(a_collisionShape, a_mass, new btDefaultMotionState(btTransform(a_startRotation, a_startPosition)))){};

	protected:
		btRigidBodyConstructionInfo defaultConstructionInfo(btCollisionShape* a_collisionShape, btScalar a_mass = 0, btMotionState* a_motionState = new btDefaultMotionState())
		{
			//rigidbody is dynamic if and only if mass is non zero, otherwise static
			bool isDynamic = (a_mass != 0.f);

			btVector3 localInertia(0, 0, 0);
			if (isDynamic)
				a_collisionShape->calculateLocalInertia(a_mass, localInertia);

			return btRigidBodyConstructionInfo(a_mass, a_motionState, a_collisionShape, localInertia);
		};
};

#endif