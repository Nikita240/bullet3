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

#include "DoubleWishboneDemo.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../CommonInterfaces/CommonParameterInterface.h"

#include "GroundShape.h"
#include "BoxShape.h"
#include "LinkShape.h"
#include "RigidBody.h"
#include "GroundBody.h"
#include "WishboneShape.h"

struct DoubleWishboneDemoExample : public CommonRigidBodyBase
{
	DoubleWishboneDemoExample(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~DoubleWishboneDemoExample() {}
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
	void createDoubleWishbone();
};

void DoubleWishboneDemoExample::setSliderParameters()
{
	// {
	// 	SliderParams slider("Tao", &gs_constraintSetting->m_tau);
	// 	slider.m_minVal = 0;
	// 	slider.m_maxVal = 10.0f;
	// 	m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	// }
}

void DoubleWishboneDemoExample::initPhysics()
{
	//setup world
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);


	//create ground
	// createGround();

	//create point constraint boxes
	createDoubleWishbone();

	//create parameter sliders
	setSliderParameters();

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

// @brief create ground
void DoubleWishboneDemoExample::createGround()
{
	GroundShape* groundShape = new GroundShape();
	m_collisionShapes.push_back(groundShape);

	m_dynamicsWorld->addRigidBody(new GroundBody(groundShape));
}

// @brief create double wishbone suspension
void DoubleWishboneDemoExample::createDoubleWishbone()
{
    // Units are in meters.

	// Disable gravity for now.
	m_dynamicsWorld->setGravity(btVector3(0, 0, 0));

	// Create suspension mounting body.
	BoxShape* suspensionMountShape = new BoxShape(btVector3(.5, .5, .5));
	m_collisionShapes.push_back(suspensionMountShape);

	btRigidBody* suspensionMount = new RigidBody(suspensionMountShape, 0);
    m_dynamicsWorld->addRigidBody(suspensionMount);
    

    // Create wishbones
	WishboneShape* upperWishboneShape = new WishboneShape(
		btVector3(  0, 0, -1),
		btVector3(-.5, 0, 0),
		btVector3( .5, 0, 0));
	m_collisionShapes.push_back(upperWishboneShape);
	WishboneShape* lowerWishboneShape = new WishboneShape(
		btVector3(0, 0, -1),
		btVector3(-.5, 0, 0),
		btVector3(.5, 0, 0));
	m_collisionShapes.push_back(lowerWishboneShape);

	btRigidBody* upperWishbone = new RigidBody(upperWishboneShape, 1);
	m_dynamicsWorld->addRigidBody(upperWishbone);
	btRigidBody* lowerWishbone = new RigidBody(lowerWishboneShape, 1);
	m_dynamicsWorld->addRigidBody(lowerWishbone);

	// Disable collisions for wishbones
	lowerWishbone->setCollisionFlags(upperWishbone->getCollisionFlags() | 4);
	lowerWishbone->setCollisionFlags(upperWishbone->getCollisionFlags() | 4);

	// Create hinge constraints
	btHingeConstraint* upperHinge = new btHingeConstraint(
		*suspensionMount,
		*upperWishbone,
		btVector3(0.5, 0.5, -0.5),  // Front mounting point
		btVector3(.5, 0, 0),        // Wishbone A point
		btVector3(1, 0, 0),         // X axis of subframe.
		btVector3(1, 0, 0)          // X axis of wishbone.
		);
	upperHinge->setLimit(-1, 1);
	m_dynamicsWorld->addConstraint(upperHinge);

	btHingeConstraint* lowerHinge = new btHingeConstraint(
		*suspensionMount,
		*lowerWishbone,
		btVector3(0.5, -0.5, -0.5),  // Front mounting point
		btVector3(.5, 0, 0),         // Wishbone A point
		btVector3(1, 0, 0),          // X axis of subframe.
		btVector3(1, 0, 0)           // X axis of wishbone.
	);
	lowerHinge->setLimit(-1, 1); 
	m_dynamicsWorld->addConstraint(lowerHinge);

	// Create knuckle
	BoxShape* knuckleShape = new BoxShape(btVector3(.5, .5, .1));
	m_collisionShapes.push_back(knuckleShape);

	btRigidBody* knuckle = new RigidBody(knuckleShape, 5);
	m_dynamicsWorld->addRigidBody(knuckle);

	// Attach knuckle to wishbones
	m_dynamicsWorld->addConstraint(new btPoint2PointConstraint(
		*knuckle,
		*upperWishbone,
		btVector3(0, -0.5, 0),  // Top balljoint
		btVector3(0, 0, -1)    // Wishbone knuckle point
		));

	m_dynamicsWorld->addConstraint(new btPoint2PointConstraint(
		*knuckle,
		*lowerWishbone,
		btVector3(0, 0.5, 0),  // Bottom balljoint
		btVector3(0, 0, -1)    // Wishbone knuckle point
		));
}



void DoubleWishboneDemoExample::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

CommonExampleInterface* DoubleWishboneDemoCreateFunc(CommonExampleOptions& options)
{
	return new DoubleWishboneDemoExample(options.m_guiHelper);
}
