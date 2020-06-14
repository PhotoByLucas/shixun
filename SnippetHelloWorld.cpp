

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

// ****************************************************************************
// This snippet illustrates simple use of physx
//
// It creates a number of box stacks on a plane, and if rendering, allows the
// user to create new stacks and fire a ball from the camera position
// ****************************************************************************

#include <ctype.h>
#include <vector>
#include "PxPhysicsAPI.h"

#include "../SnippetCommon/SnippetPrint.h"
#include "../SnippetCommon/SnippetPVD.h"
#include "../SnippetUtils/SnippetUtils.h"


using namespace physx;




PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation*			gFoundation = NULL;
PxPhysics*				gPhysics	= NULL;

PxDefaultCpuDispatcher*	gDispatcher = NULL;
PxScene*				gScene		= NULL;

PxMaterial*				gMaterial	= NULL;

PxPvd*                  gPvd        = NULL;


PxRigidStatic* plane;

std::vector<PxVec3> gContactPositions;
std::vector<PxVec3> gContactImpulses;


PxReal stackZ = 10.0f;

PxFilterFlags contactReportFilterShader(PxFilterObjectAttributes attributes0, PxFilterData filterData0,
	PxFilterObjectAttributes attributes1, PxFilterData filterData1,
	PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
	PX_UNUSED(attributes0);
	PX_UNUSED(attributes1);
	PX_UNUSED(filterData0);
	PX_UNUSED(filterData1);
	PX_UNUSED(constantBlockSize);
	PX_UNUSED(constantBlock);

	// all initial and persisting reports for everything, with per-point data
	pairFlags = PxPairFlag::eSOLVE_CONTACT | PxPairFlag::eDETECT_DISCRETE_CONTACT
		| PxPairFlag::eNOTIFY_TOUCH_FOUND
		| PxPairFlag::eNOTIFY_TOUCH_PERSISTS
		| PxPairFlag::eNOTIFY_CONTACT_POINTS;
	return PxFilterFlag::eDEFAULT;
}


class ContactReportCallback : public PxSimulationEventCallback
{
	void onConstraintBreak(PxConstraintInfo* constraints, PxU32 count) { PX_UNUSED(constraints); PX_UNUSED(count); }
	void onWake(PxActor** actors, PxU32 count) { PX_UNUSED(actors); PX_UNUSED(count); }
	void onSleep(PxActor** actors, PxU32 count) { PX_UNUSED(actors); PX_UNUSED(count); }
	void onTrigger(PxTriggerPair* pairs, PxU32 count) { PX_UNUSED(pairs); PX_UNUSED(count); }
	void onAdvance(const PxRigidBody*const*, const PxTransform*, const PxU32) {}
	void onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs)
	{
		PX_UNUSED((pairHeader));
		std::vector<PxContactPairPoint> contactPoints;

		for (PxU32 i = 0; i < nbPairs; i++)
		{
			PxU32 contactCount = pairs[i].contactCount;
			if (contactCount)
			{
				contactPoints.resize(contactCount);
				pairs[i].extractContacts(&contactPoints[0], contactCount);

				for (PxU32 j = 0; j < contactCount; j++)
				{
					gContactPositions.push_back(contactPoints[j].position);
					gContactImpulses.push_back(contactPoints[j].impulse);
				}
			}
		}
	}
};

ContactReportCallback gContactReportCallback;

PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity=PxVec3(100))
{

	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	
	//dynamic->setAngularVelocity(velocity);
	gScene->addActor(*dynamic);
	

	return dynamic;
}
/***
void moveLeft(const PxRigidDynamic& left, const PxGeometry& geometry, const PxVec3& velocity = PxVec3(100)) {
	
}
***/

//�����ؽ�
PxJoint* createMyJoint(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1) {
	PxRevoluteJoint* j = PxRevoluteJointCreate(*gPhysics, a0, t0, a1, t1);
	j->setProjectionLinearTolerance(0.1f);
	j->setConstraintFlag(PxConstraintFlag::ePROJECTION, true);
	return j;
}
PxJoint* createDampedD6(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1)
{
	PxD6Joint* j = PxD6JointCreate(*gPhysics, a0, t0, a1, t1);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
	//j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
	//j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
	j->setProjectionLinearTolerance(0.1f);
	j->setConstraintFlag(PxConstraintFlag::ePROJECTION, true);
	j->setDrive(PxD6Drive::eSLERP, PxD6JointDrive(0, 1000, FLT_MAX, true));
	return j;
}
typedef PxJoint* (*JointCreateFunction)(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1);
void createChain(const PxTransform& t, PxU32 length, const PxGeometry& g, PxReal separation, JointCreateFunction createJoint)
{
	PxVec3 offset(separation / 2, 0, 0);
	PxTransform localTm(offset);
	PxRigidDynamic* prev = NULL;
	for (PxU32 i = 0; i < length; i++)
	{
		PxRigidDynamic* current = PxCreateDynamic(*gPhysics, t*localTm, g, *gMaterial, 1.0f);
		(*createJoint)(prev, prev ? PxTransform(offset) : t, current, PxTransform(-offset));
		gScene->addActor(*current);
		prev = current;
		localTm.p.x += separation;
	}
}


//���������
PxRigidDynamic* createBall(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity = PxVec3(100))
{

	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	dynamic->setRigidDynamicLockFlags(PxRigidDynamicLockFlag::eLOCK_LINEAR_Y);
	//dynamic->setAngularVelocity(velocity);
	gScene->addActor(*dynamic);


	return dynamic;
}

void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent)
{
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);
	for(PxU32 i=0; i<size;i++)
	{
		for(PxU32 j=0;j<size-i;j++)
		{
			PxTransform localTm(PxVec3(PxReal(j*2) - PxReal(size-i), PxReal(i*2+1), 0) * halfExtent);
			PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
			body->attachShape(*shape);
			PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
			gScene->addActor(*body);
		}
	}
	
	shape->release();
}



void initPhysics(bool interactive)
{
	gFoundation = PxCreateFoundation(PX_FOUNDATION_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(),true,gPvd);
	PxInitExtensions(*gPhysics, gPvd);
	//PxU32 numCores = SnippetUtils::getNbPhysicalCores();

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 5.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= contactReportFilterShader;
	sceneDesc.simulationEventCallback = &gContactReportCallback;
	gScene = gPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.98f, 0.98f);


	//����Χǽ
	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
	gScene->addActor(*groundPlane);
	PxShape* wallShape1 = gPhysics->createShape(PxBoxGeometry(100.0f,10.0f,1.0f), *gMaterial);
	PxRigidStatic* southWall = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.0f, 10.0f, 200.0f)), *wallShape1);
	PxRigidStatic* northWall = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.0f, 10.0f, -200.0f)), *wallShape1);
	PxShape* wallShape2 = gPhysics->createShape(PxBoxGeometry(1.0f, 10.0f, 200.0f), *gMaterial);
	PxRigidStatic* eastWall = PxCreateStatic(*gPhysics, PxTransform(PxVec3(100.0f, 10.0f, 0.0f)), *wallShape2);
	PxRigidStatic* westWall = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-100.0f, 10.0f, 0.0f)), *wallShape2);
	gScene->addActor(*southWall);
	gScene->addActor(*westWall);
	gScene->addActor(*eastWall);
	gScene->addActor(*northWall);

	//Сǽ�����
	PxShape* wallShape3 = gPhysics->createShape(PxBoxGeometry(1.0f, 10.0f, 160.0f), *gMaterial);
	PxRigidStatic* smallWall = PxCreateStatic(*gPhysics, PxTransform(PxVec3(90.0f, 10.0f, 40.0f)), *wallShape3);
	gScene->addActor(*smallWall);

	//PxShape* shape4 = gPhysics->createShape(PxBoxGeometry(1.0f, 10.0f, 100.0f), *gMaterial);
	//PxRigidStatic* leftStick = PxCreateStatic(*gPhysics, PxTransform(PxVec3(100.0f, 10.0f, 0.0f)), *shape2);

	//PxRigidStatic* rightStick = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-100.0f, 10.0f, 0.0f)), *shape2);


	//������
	//PxRigidDynamic* ball = createDynamic(PxTransform(PxVec3(95.0f, 0.0f, 180.0f)), PxSphereGeometry(4.0f), PxVec3(0.0f, 0.0f, 0.0f));
	//ball->setRigidDynamicLockFlags( PxRigidDynamicLockFlag::eLOCK_LINEAR_Y);
	//gScene->addActor(*ball);

	//�����ϰ�
	PxShape* capsuleShape = gPhysics->createShape(PxCapsuleGeometry(10.0f, 20.0f), *gMaterial);
	//PxRigidStatic* stick0 = gPhysics->createRigidStatic(PxTransform(PxVec3(10.0f, 0.0f, -70.0f)));
	PxTransform relativePose(PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
	capsuleShape->setLocalPose(relativePose);

	PxRigidStatic* stick1 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.0f, 0.0f, 0.0f)), *capsuleShape);
	//stick->setRigidDynamicLockFlags(PxRigidDynamicLockFlag::eLOCK_LINEAR_Y);
	gScene->addActor(*stick1);
	PxRigidStatic* stick2 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.0f, 0.0f, 50.0f)), *capsuleShape);
	//stick->setRigidDynamicLockFlags(PxRigidDynamicLockFlag::eLOCK_LINEAR_Y);
	gScene->addActor(*stick2);
	PxRigidStatic* stick3 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.0f, 0.0f, -50.0f)), *capsuleShape);
	//stick->setRigidDynamicLockFlags(PxRigidDynamicLockFlag::eLOCK_LINEAR_Y);
	gScene->addActor(*stick3);

	PxTransform relativePose6(PxQuat(PxHalfPi, PxVec3(0, 1, 0)));
	PxShape* capsuleShape1 = gPhysics->createShape(PxCapsuleGeometry(10.0f, 10.0f), *gMaterial);
	capsuleShape1->setLocalPose(relativePose6);
	PxRigidStatic* capsule1 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(70.0f, 0.0f, 70.0f)), *capsuleShape1);
	//stick->setRigidDynamicLockFlags(PxRigidDynamicLockFlag::eLOCK_LINEAR_Y);
	gScene->addActor(*capsule1);
	PxRigidStatic* capsule2 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-80.0f, 0.0f, 70.0f)), *capsuleShape1);
	gScene->addActor(*capsule2);
	PxShape* boxShape = gPhysics->createShape(PxBoxGeometry(10.0f, 10.0f, 10.0f), *gMaterial);
	PxRigidStatic* box = PxCreateStatic(*gPhysics, PxTransform(PxVec3(40.0f, 10.0f, 50.0f)), *boxShape);
	gScene->addActor(*box);
	PxRigidStatic* box1 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-40.0f, 10.0f, 50.0f)), *boxShape);
	gScene->addActor(*box1);
	PxShape* boxShape1 = gPhysics->createShape(PxBoxGeometry(10.0f, 10.0f, 10.0f), *gMaterial);
	PxTransform relativePose5(PxQuat(PxHalfPi*0.5, PxVec3(0, 1, 0)));
	boxShape1->setLocalPose(relativePose5);
	PxRigidStatic* box2 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-50.0f, 10.0f, -20.0f)), *boxShape1);
	gScene->addActor(*box2);
	PxRigidStatic* box3 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(50.0f, 10.0f, -20.0f)), *boxShape1);
	gScene->addActor(*box3);
	
	
	//�·��ұߵ����
	PxShape* rightHandWall = gPhysics->createShape(PxBoxGeometry(50.0f, 10.0f, 1.0f), *gMaterial);
	PxTransform relativePose1(PxQuat(PxHalfPi*0.33, PxVec3(0, 1, 0)));
	rightHandWall->setLocalPose(relativePose1);
	PxRigidStatic* stick4 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(45.0f, 10.0f, 120.0f)), *rightHandWall);
	gScene->addActor(*stick4);
	//�����ùؽ�ʵ�ְڱ�
	createChain(PxTransform(PxVec3(90.0f, 30.0f, 95.0f)), 2, PxBoxGeometry(25.0f, 10.0f, 1.0f), 49.0f, createDampedD6);
	
	//�·���ߵ����
	PxShape* leftHandWall = gPhysics->createShape(PxBoxGeometry(50.0f, 10.0f, 1.0f), *gMaterial);
	PxTransform relativePose2(PxQuat(-0.33*PxHalfPi, PxVec3(0, 1, 0)));
	leftHandWall->setLocalPose(relativePose2);
	PxRigidStatic* stick5 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-55.0f, 10.0f, 120.0f)), *leftHandWall);
	gScene->addActor(*stick5);
	//stick->setRigidDynamicLockFlags(PxRigidDynamicLockFlag::eLOCK_LINEAR_Y);

	//�Ϸ���ߵ����
	PxShape* TopLeftWall = gPhysics->createShape(PxBoxGeometry(55.0f, 10.0f, 1.0f), *gMaterial);
	PxTransform relativePose3(PxQuat(PxHalfPi*0.33, PxVec3(0, 1, 0)));
	TopLeftWall->setLocalPose(relativePose3);
	PxRigidStatic* stick6 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-52.5f, 10.0f, -175.0f)), *TopLeftWall);
	gScene->addActor(*stick6);

	//�Ϸ��ұߵ����
	PxShape* TopRightWall = gPhysics->createShape(PxBoxGeometry(50.0f, 10.0f, 1.0f), *gMaterial);
	PxTransform relativePose4(PxQuat(-0.33*PxHalfPi, PxVec3(0, 1, 0)));
	TopRightWall->setLocalPose(relativePose4);
	PxRigidStatic* stick7 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(55.0f, 10.0f, -175.0f)), *TopRightWall);
	gScene->addActor(*stick7);





	//PxShape* triShape = gPhysics->createShape(PxTriangleGeometry(10.0f,10.f,10.0f),*gMaterial);
	//PxRigidStatic* tri1 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.0f, 0.0f, 0.0f)), *triShape);
	//for(PxU32 i=0;i<5;i++)
	//createStack(PxTransform(PxVec3(0,0,stackZ-=10.0f)), 10, 2.0f);



	//����С��
	if(!interactive)
		createDynamic(PxTransform(PxVec3(0,40,100)), PxSphereGeometry(10), PxVec3(0,0,0.1f));
}

//��ͼ��ʼ��
void createMap() {

}

void stepPhysics(bool interactive)
{
	PX_UNUSED(interactive);
	gContactPositions.clear();
	gContactImpulses.clear();

	gScene->simulate(1.0f/60.0f);
	gScene->fetchResults(true);
	//����ָ������֮��gScene->removeActor()С��
	printf("%d contact reports\n", PxU32(gContactPositions.size()));
	
}
	
void cleanupPhysics(bool interactive)
{
	PX_UNUSED(interactive);
	gScene->release();
	gDispatcher->release();
	PxCloseExtensions();
	gPhysics->release();	
	PxPvdTransport* transport = gPvd->getTransport();
	gPvd->release();
	transport->release();
	
	gFoundation->release();
	
	printf("SnippetHelloWorld done.\n");
}

void keyPress(unsigned char key, const PxTransform& camera)
{
	switch(toupper(key))
	{
	case 'B':	createStack(PxTransform(PxVec3(0,0,stackZ-=10.0f)), 10, 2.0f);						break;
	case ' ':	createDynamic(camera, PxSphereGeometry(4.0f), camera.rotate(PxVec3(0,0,-1))*200);	break;
	case 'T':	createBall(PxTransform(PxVec3(95.0f, 2.0f, 184.0f)), PxSphereGeometry(4.0f), PxVec3(0, 0, -100));	break;
	case 'Q':;//��ڱ�
	case 'E':;//�Ұڱ�

	}
}



int snippetMain(int, const char*const*)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for(PxU32 i=0; i<frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}
