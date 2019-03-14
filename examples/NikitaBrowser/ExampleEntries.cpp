#include "ExampleEntries.h"

#include "../BlockSolver/BlockSolverExample.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "EmptyExample.h"
#include "../BulletRobotics/BoxStack.h"
#include "../BulletRobotics/FixJointBoxes.h"
#include "../BulletRobotics/JointLimit.h"
// #include "../BulletRobotics/GraspBox.h"
#include "../BulletRobotics/FixJointBoxes.h"
#include "../RenderingExamples/RenderInstancingDemo.h"
#include "../RenderingExamples/CoordinateSystemDemo.h"
#include "../RenderingExamples/RaytracerSetup.h"
#include "../RenderingExamples/TinyRendererSetup.h"
#include "../RenderingExamples/DynamicTexturedCubeDemo.h"
#include "../ForkLift/ForkLiftDemo.h"
#include "../MultiThreadedDemo/MultiThreadedDemo.h"
#include "../BasicDemo/BasicExample.h"
#include "../Planar2D/Planar2D.h"
#include "../Benchmarks/BenchmarkDemo.h"
#include "../Importers/ImportObjDemo/ImportObjExample.h"
#include "../Importers/ImportBsp/ImportBspExample.h"
#include "../Importers/ImportColladaDemo/ImportColladaSetup.h"
#include "../Importers/ImportSTLDemo/ImportSTLSetup.h"
#include "../Importers/ImportURDFDemo/ImportURDFSetup.h"
#include "../Importers/ImportSDFDemo/ImportSDFSetup.h"
#include "../Importers/ImportMJCFDemo/ImportMJCFSetup.h"
#include "../Collision/CollisionTutorialBullet2.h"
#include "../GyroscopicDemo/GyroscopicSetup.h"
#include "../Constraints/Dof6Spring2Setup.h"
#include "../Constraints/ConstraintPhysicsSetup.h"
#include "../MultiBody/TestJointTorqueSetup.h"
#include "../MultiBody/Pendulum.h"
#include "../MultiBody/MultiBodySoftContact.h"
#include "../MultiBody/MultiBodyConstraintFeedback.h"
#include "../MultiBody/MultiDofDemo.h"
#include "../MultiBody/InvertedPendulumPDControl.h"

#include "../RigidBody/RigidBodySoftContact.h"
#include "../VoronoiFracture/VoronoiFractureDemo.h"
#include "../SoftDemo/SoftDemo.h"
#include "../Constraints/ConstraintDemo.h"
#include "../Vehicles/Hinge2Vehicle.h"
#include "../Importers/ImportBullet/SerializeSetup.h"
#include "../Raycast/RaytestDemo.h"
#include "../FractureDemo/FractureDemo.h"
#include "../DynamicControlDemo/MotorDemo.h"
#include "../RollingFrictionDemo/RollingFrictionDemo.h"
#include "../SharedMemory/PhysicsServerExampleBullet2.h"
#include "../SharedMemory/PhysicsServerExample.h"
#include "../SharedMemory/PhysicsClientExample.h"
#include "../Constraints/TestHingeTorque.h"
#include "../RenderingExamples/TimeSeriesExample.h"
#include "../Tutorial/Tutorial.h"
#include "../Tutorial/Dof6ConstraintTutorial.h"
#include "../MultiThreading/MultiThreadingExample.h"
#include "../InverseDynamics/InverseDynamicsExample.h"
#include "../RoboticsLearning/R2D2GraspExample.h"
#include "../RoboticsLearning/KukaGraspExample.h"
#include "../RoboticsLearning/GripperGraspExample.h"
#include "../InverseKinematics/InverseKinematicsExample.h"

#ifdef B3_ENABLE_TINY_AUDIO
#include "../TinyAudio/TinyAudioExample.h"
#endif  //B3_ENABLE_TINY_AUDIO

#ifdef ENABLE_LUA
#include "../LuaDemo/LuaPhysicsSetup.h"
#endif

#ifdef B3_USE_CLEW
#ifndef NO_OPENGL3
#include "../OpenCL/broadphase/PairBench.h"
#include "../OpenCL/rigidbody/GpuConvexScene.h"
#endif
#endif  //B3_USE_CLEW

//Extended Tutorial Includes Added by Mobeen and Benelot
#include "../ExtendedTutorials/SimpleBox.h"
#include "../ExtendedTutorials/MultipleBoxes.h"
#include "../ExtendedTutorials/CompoundBoxes.h"
#include "../ExtendedTutorials/SimpleJoint.h"
#include "../ExtendedTutorials/SimpleCloth.h"
#include "../ExtendedTutorials/Chain.h"
#include "../ExtendedTutorials/Bridge.h"
#include "../ExtendedTutorials/RigidBodyFromObj.h"
#include "../ExtendedTutorials/InclinedPlane.h"
#include "../ExtendedTutorials/NewtonsCradle.h"
#include "../ExtendedTutorials/NewtonsRopeCradle.h"
#include "../ExtendedTutorials/MultiPendulum.h"
#include "../Evolution/NN3DWalkers.h"

//Nikita's Demo
#include "../Nikita/BoxDemo.h"
#include "../Nikita/PointConstraintDemo.h"
#include "../Nikita/DoubleWishboneDemo.h"

struct ExampleEntry
{
	int m_menuLevel;
	const char* m_name;
	const char* m_description;
	CommonExampleInterface::CreateFunc* m_createFunc;
	int m_option;

	ExampleEntry(int menuLevel, const char* name)
		: m_menuLevel(menuLevel), m_name(name), m_description(0), m_createFunc(0), m_option(0)
	{
	}

	ExampleEntry(int menuLevel, const char* name, const char* description, CommonExampleInterface::CreateFunc* createFunc, int option = 0)
		: m_menuLevel(menuLevel), m_name(name), m_description(description), m_createFunc(createFunc), m_option(option)
	{
	}
};

static ExampleEntry gDefaultExamples[] =
	{
		ExampleEntry(0, "Nikita Demo"),

		ExampleEntry(1, "Box Demo", "Basic Box Demo.", BoxDemoCreateFunc),

		ExampleEntry(1, "Point Constraint Demo", "Demonstation of Point Constraints with adjustable parameters.", PointConstraintDemoCreateFunc),

		ExampleEntry(1, "Double Wishbone Demo", "Demonstation of Double Wishbone suspension.", DoubleWishboneDemoCreateFunc),
};

#ifdef B3_USE_CLEW
#ifndef NO_OPENGL3
static ExampleEntry gOpenCLExamples[] =
	{
		

};
#endif
#endif  //
static btAlignedObjectArray<ExampleEntry> gAdditionalRegisteredExamples;

struct ExampleEntriesInternalData
{
	btAlignedObjectArray<ExampleEntry> m_allExamples;
};

ExampleEntriesAll::ExampleEntriesAll()
{
	m_data = new ExampleEntriesInternalData;
}

ExampleEntriesAll::~ExampleEntriesAll()
{
	delete m_data;
}

void ExampleEntriesAll::initOpenCLExampleEntries()
{
#ifdef B3_USE_CLEW
#ifndef NO_OPENGL3
	int numDefaultEntries = sizeof(gOpenCLExamples) / sizeof(ExampleEntry);
	for (int i = 0; i < numDefaultEntries; i++)
	{
		m_data->m_allExamples.push_back(gOpenCLExamples[i]);
	}
#endif
#endif  //B3_USE_CLEW
}

void ExampleEntriesAll::initExampleEntries()
{
	m_data->m_allExamples.clear();

	for (int i = 0; i < gAdditionalRegisteredExamples.size(); i++)
	{
		m_data->m_allExamples.push_back(gAdditionalRegisteredExamples[i]);
	}

	int numDefaultEntries = sizeof(gDefaultExamples) / sizeof(ExampleEntry);
	for (int i = 0; i < numDefaultEntries; i++)
	{
		m_data->m_allExamples.push_back(gDefaultExamples[i]);
	}

	if (m_data->m_allExamples.size() == 0)
	{
		{
			ExampleEntry e(0, "Empty");
			m_data->m_allExamples.push_back(e);
		}

		{
			ExampleEntry e(1, "Empty", "Empty Description", EmptyExample::CreateFunc);
			m_data->m_allExamples.push_back(e);
		}
	}
}

void ExampleEntriesAll::registerExampleEntry(int menuLevel, const char* name, const char* description, CommonExampleInterface::CreateFunc* createFunc, int option)
{
	ExampleEntry e(menuLevel, name, description, createFunc, option);
	gAdditionalRegisteredExamples.push_back(e);
}

int ExampleEntriesAll::getNumRegisteredExamples()
{
	return m_data->m_allExamples.size();
}

CommonExampleInterface::CreateFunc* ExampleEntriesAll::getExampleCreateFunc(int index)
{
	return m_data->m_allExamples[index].m_createFunc;
}

int ExampleEntriesAll::getExampleOption(int index)
{
	return m_data->m_allExamples[index].m_option;
}

const char* ExampleEntriesAll::getExampleName(int index)
{
	return m_data->m_allExamples[index].m_name;
}

const char* ExampleEntriesAll::getExampleDescription(int index)
{
	return m_data->m_allExamples[index].m_description;
}
