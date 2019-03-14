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

#include "PointConstraintDemo.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../CommonInterfaces/CommonParameterInterface.h"

#include "GroundShape.h"
#include "BoxShape.h"
#include "RigidBody.h"
#include "GroundBody.h"

static btConstraintSetting* gs_constraintSetting;

struct PointConstraintDemoExample : public CommonRigidBodyBase
{
	PointConstraintDemoExample(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~PointConstraintDemoExample() {}
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
    void setSliderParameters();
    // void setConstraintSettings(btScalar tau);

	void createGround();
	void createBasicBox();
	void createPointConstraintedBoxes();

	btPoint2PointConstraint* m_pointConstraint;
};

void PointConstraintDemoExample::setSliderParameters()
{
	{
		SliderParams slider("Tao", &gs_constraintSetting->m_tau);
		slider.m_minVal = 0;
		slider.m_maxVal = 10.0f;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}

	{
		SliderParams slider("Damping", &gs_constraintSetting->m_damping);
		slider.m_minVal = 0;
		slider.m_maxVal = 10.0f;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}

	{
		SliderParams slider("Impulse Clamp", &gs_constraintSetting->m_impulseClamp);
		slider.m_minVal = 0;
		slider.m_maxVal = 10.0f;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}
}

void PointConstraintDemoExample::initPhysics()
{
	//setup world
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);


	//create ground
	createGround();

	//create point constraint boxes
	createPointConstraintedBoxes();

	//create parameter sliders
	setSliderParameters();

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

// @brief create ground
void PointConstraintDemoExample::createGround()
{
	GroundShape* groundShape = new GroundShape();
	m_collisionShapes.push_back(groundShape);

	m_dynamicsWorld->addRigidBody(new GroundBody(groundShape));
}

// @brief create two point constrained boxes.
void PointConstraintDemoExample::createPointConstraintedBoxes()
{
	// Re-using the same collision is better for memory usage and performance
	BoxShape* boxShape = new BoxShape();
	m_collisionShapes.push_back(boxShape);

	// Create first box
	RigidBody* boxA = new RigidBody(boxShape, 0.1, btVector3(0, 2, 0));
	m_dynamicsWorld->addRigidBody(boxA);

	// Create second box
	RigidBody* boxB = new RigidBody(boxShape, 0.1, btVector3(0, 1, 0));
	m_dynamicsWorld->addRigidBody(boxB);

	// Create point constraint
	m_pointConstraint = new btPoint2PointConstraint(
		*boxA,
		*boxB,
		btVector3(0, -0.1, 0),  // Bottom of box A.
		btVector3(0, 0.3, 0)    // Top of box B + 1 box widths.
	);
	gs_constraintSetting = &m_pointConstraint->m_setting;
	m_dynamicsWorld->addConstraint(m_pointConstraint);
}



void PointConstraintDemoExample::renderScene()
{
	CommonRigidBodyBase::renderScene();

	printf("Tao: %f Damping: %f Impulse Clamp: %f\n",
		   m_pointConstraint->m_setting.m_tau,
		   m_pointConstraint->m_setting.m_damping,
		   m_pointConstraint->m_setting.m_impulseClamp
        );
}

CommonExampleInterface* PointConstraintDemoCreateFunc(CommonExampleOptions& options)
{
	return new PointConstraintDemoExample(options.m_guiHelper);
}
