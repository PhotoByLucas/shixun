// ÎÒÉ±ÄãÂð

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


PxReal stackZ = 10.0f;

PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity=PxVec3(0))
{

	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
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

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 1.0f);


	//ÉèÖÃÎ§Ç½
	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
	gScene->addActor(*groundPlane);
	PxShape* shape1 = gPhysics->createShape(PxBoxGeometry(100.0f,10.0f,1.0f), *gMaterial);
	PxRigidStatic* southWall = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.0f, 10.0f, 100.0f)), *shape1);
	PxRigidStatic* northWall = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.0f, 10.0f, -100.0f)), *shape1);
	PxShape* shape2 = gPhysics->createShape(PxBoxGeometry(1.0f, 10.0f, 100.0f), *gMaterial);
	PxRigidStatic* westWall = PxCreateStatic(*gPhysics, PxTransform(PxVec3(100.0f, 10.0f, 0.0f)), *shape2);
	PxRigidStatic* eastWall = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-100.0f, 10.0f, 0.0f)), *shape2);
	gScene->addActor(*southWall);
	gScene->addActor(*westWall);
	gScene->addActor(*eastWall);
	gScene->addActor(*northWall);
	
	//PxShape* shape4 = gPhysics->createShape(PxBoxGeometry(1.0f, 10.0f, 100.0f), *gMaterial);
	//PxRigidStatic* leftStick = PxCreateStatic(*gPhysics, PxTransform(PxVec3(100.0f, 10.0f, 0.0f)), *shape2);

	//PxRigidStatic* rightStick = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-100.0f, 10.0f, 0.0f)), *shape2);


	//Éú³ÉÇò
	PxRigidDynamic* ball = createDynamic(PxTransform(PxVec3(40.0f, 10.0f, 0.0f)), PxSphereGeometry(10.0f), PxVec3(0.0f, 0.0f, 0.0f));
	ball->setRigidDynamicLockFlags( PxRigidDynamicLockFlag::eLOCK_LINEAR_Y);
	//ball->addForce()
	gScene->addActor(*ball);
	//½ºÄÒÕÏ°­
	PxShape* shape3 = gPhysics->createShape(PxCapsuleGeometry(10.0f, 20.0f), *gMaterial);
	//PxRigidStatic* stick0 = gPhysics->createRigidStatic(PxTransform(PxVec3(10.0f, 0.0f, -70.0f)));
	PxTransform relativePose(PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
	shape3->setLocalPose(relativePose);
	PxRigidStatic* stick1 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.0f, 0.0f, 10.0f)), *shape3);
	//stick->setRigidDynamicLockFlags(PxRigidDynamicLockFlag::eLOCK_LINEAR_Y);
	gScene->addActor(*stick1);
	PxRigidStatic* stick2 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.0f, 0.0f, 50.0f)), *shape3);
	//stick->setRigidDynamicLockFlags(PxRigidDynamicLockFlag::eLOCK_LINEAR_Y);
	gScene->addActor(*stick2);
	PxRigidStatic* stick3 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.0f, 0.0f, -50.0f)), *shape3);
	//stick->setRigidDynamicLockFlags(PxRigidDynamicLockFlag::eLOCK_LINEAR_Y);
	gScene->addActor(*stick3);
	PxShape* shape4 = gPhysics->createShape(PxBoxGeometry(10.0f, 10.0f, 10.0f), *gMaterial);
	PxRigidStatic* box = PxCreateStatic(*gPhysics, PxTransform(PxVec3(50.0f, 20.0f, 50.0f)), *shape4);
	gScene->addActor(*box);
	//for(PxU32 i=0;i<5;i++)
		//createStack(PxTransform(PxVec3(0,0,stackZ-=10.0f)), 10, 2.0f);



	//·¢ÉäÐ¡Çò
	if(!interactive)
		createDynamic(PxTransform(PxVec3(0,40,100)), PxSphereGeometry(10), PxVec3(0,0,0.1f));
}

//µØÍ¼³õÊ¼»¯
void createMap() {

}

void stepPhysics(bool interactive)
{
	PX_UNUSED(interactive);
	gScene->simulate(1.0f/60.0f);
	gScene->fetchResults(true);
}
	
void cleanupPhysics(bool interactive)
{
	PX_UNUSED(interactive);
	gScene->release();
	gDispatcher->release();
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
	case ' ':	createDynamic(camera, PxSphereGeometry(3.0f), camera.rotate(PxVec3(0,0,-1))*200);	break;
	case 'Q':;//×ó°Ú±Û
	case 'E':;//ÓÒ°Ú±Û
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
