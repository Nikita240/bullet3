/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "BoxDemo.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"

#include "GroundShape.h"
#include "BoxShape.h"
#include "RigidBody.h"
#include "GroundBody.h"

struct BoxDemoExample : public CommonRigidBodyBase
{
	BoxDemoExample(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~BoxDemoExample() {}
	virtual void initPhysics();
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 4.1;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3] = {0, 0.46, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}

    private:
        void createGround();
        void createBasicBox();
		void createPointConstraintedBoxes();
};

void BoxDemoExample::initPhysics()
{
	//setup world
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);


	//create ground
	createGround();

    //create basic box
	createBasicBox();

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

// @brief create ground
void BoxDemoExample::createGround()
{
	GroundShape* groundShape = new GroundShape();
	m_collisionShapes.push_back(groundShape);

	m_dynamicsWorld->addRigidBody(new GroundBody(groundShape));
}

// @brief create basix box
void BoxDemoExample::createBasicBox()
{
	BoxShape* boxShape = new BoxShape();
	m_collisionShapes.push_back(boxShape);

	RigidBody* box = new RigidBody(boxShape, 0.1, btVector3(0, 2, 0));
	m_dynamicsWorld->addRigidBody(box);
}

void BoxDemoExample::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

CommonExampleInterface* BoxDemoCreateFunc(CommonExampleOptions& options)
{
	return new BoxDemoExample(options.m_guiHelper);
}
