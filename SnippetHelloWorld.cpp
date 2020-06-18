// 谁是内鬼大作战
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

#include "foundation/PxMemory.h"


#include "../SnippetCommon/SnippetPrint.h"
#include "../SnippetCommon/SnippetPVD.h"
#include "../SnippetUtils/SnippetUtils.h"
#include "PsArray.h"
#include "PxImmediateMode.h"
#include "extensions/PxMassProperties.h"

#include "score.h"
#include "math.h"
#include <iostream>
#include <vector>

using namespace physx;
using namespace std;




PxCooking*				gCooking = NULL;
PxDefaultAllocator	gAllocator;
PxDefaultErrorCallback gErrorCallback;

PxFoundation*	gFoundation = NULL;
PxPhysics*	gPhysics	= NULL;

PxDefaultCpuDispatcher*	gDispatcher = NULL;
PxScene*	gScene= NULL;


PxMaterial*	gMaterial	= NULL;

PxPvd* gPvd= NULL;

PxRigidDynamic* dynamicBall = NULL;

PxRigidStatic* plane;



std::vector<PxVec3> gContactPositions;
std::vector<PxVec3> gContactImpulses;

PxRigidDynamic* current = NULL;
PxRigidDynamic* current1 = NULL;

PxReal stackZ = 10.0f;


PxVec3 test_barrier[] = {
	PxVec3(0,0,0),
	PxVec3(10,0,0),
	PxVec3(-5,0,sqrt(75)),
	PxVec3(0,0,sqrt(300)),
	PxVec3(15,0,sqrt(75)),
	PxVec3(10,0,sqrt(300)),

	PxVec3(0,10,0),
	PxVec3(10,10,0),
	PxVec3(-5,10,sqrt(75)),
	PxVec3(0,10,sqrt(300)),
	PxVec3(15,10,sqrt(75)),
	PxVec3(10,10,sqrt(300))
};


//根据凸多面体顶点创建
PxConvexMesh* createConvexMesh(const PxVec3* verts, const PxU32 numVerts, PxPhysics& physics, PxCooking& cooking)
{
	// Create descriptor for convex mesh
	PxConvexMeshDesc convexDesc;
	convexDesc.points.count = numVerts;
	convexDesc.points.stride = sizeof(PxVec3);
	convexDesc.points.data = verts;
	convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

	PxConvexMesh* convexMesh = NULL;
	PxDefaultMemoryOutputStream buf;
	//cookConvexMesh用于第一次创建时的调用 可以序列化缓存到本地以便进行高效的探测计算
	//createConvexMesh功能相同但不会缓存 可用于不参与频繁计算的片元
	if (cooking.cookConvexMesh(convexDesc, buf))
	{
		PxDefaultMemoryInputData id(buf.getData(), buf.getSize());//从序列化数据读取内容
		convexMesh = physics.createConvexMesh(id);//通过读取的内容在PxPhysics内建立一个片元
	}
	return convexMesh;
}

PxFilterFlags ballFilterShader(
	PxFilterObjectAttributes attributes0, PxFilterData filterData0,
	PxFilterObjectAttributes attributes1, PxFilterData filterData1,
	PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
	// let triggers through
	if (PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
	{
		pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
		return PxFilterFlag::eDEFAULT;
	}
	// generate contacts for all that were not filtered above
	pairFlags = PxPairFlag::eCONTACT_DEFAULT;

	// trigger the contact callback for pairs (A,B) where 
	// the filtermask of A contains the ID of B and vice versa.
	if ((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))

	{
		score += 5;
		pairFlags|= PxPairFlag::eNOTIFY_TOUCH_FOUND;
	}

	if ((filterData0.word0 & filterData1.word2) && (filterData1.word0 & filterData0.word2))

	{
		gScene->removeActor(*dynamicBall);
		isBall = false;
		pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
	}
	return PxFilterFlag::eDEFAULT;
}

static const PxFilterData collisionGroupBall(1, 1, 1, 1);
static const PxFilterData collisionGroupObstacle(1, 1, 0, 1);
static const PxFilterData collisionGroupSouthWall(1, 1, 1, 1);

//class ContactReportCallback : public PxSimulationEventCallback
//{
//	void onConstraintBreak(PxConstraintInfo* constraints, PxU32 count) { PX_UNUSED(constraints); PX_UNUSED(count); }
//	void onWake(PxActor** actors, PxU32 count) { PX_UNUSED(actors); PX_UNUSED(count); }
//	void onSleep(PxActor** actors, PxU32 count) { PX_UNUSED(actors); PX_UNUSED(count); }
//	void onTrigger(PxTriggerPair* pairs, PxU32 count) { PX_UNUSED(pairs); PX_UNUSED(count); }
//	void onAdvance(const PxRigidBody*const*, const PxTransform*, const PxU32) {}
//	void onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs)
//	{
//		PX_UNUSED((pairHeader));
//		std::vector<PxContactPairPoint> contactPoints;
//
//		for (PxU32 i = 0; i < nbPairs; i++)
//		{
//			PxU32 contactCount = pairs[i].contactCount;
//			if (contactCount)
//			{
//				contactPoints.resize(contactCount);
//				pairs[i].extractContacts(&contactPoints[0], contactCount);
//
//				for (PxU32 j = 0; j < contactCount; j++)
//				{
//					gContactPositions.push_back(contactPoints[j].position);
//					gContactImpulses.push_back(contactPoints[j].impulse);
//				}
//			}
//		}
//	}
//};

//ContactReportCallback gContactReportCallback;

PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity=PxVec3(100))
{

	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	
	//dynamic->setAngularVelocity(velocity);
	gScene->addActor(*dynamic);
	

	return dynamic;
}

int score = 0;
//增加分数
void increaseScore()
{
	score++;
	cout << score << endl;
}

void moveLeft(PxRigidDynamic* left) {
	//left->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
	//left->addTorque(PxVec3(0, 0, 1000000), PxForceMode::eFORCE, true);
	left->addForce(PxVec3(-100000000.0f, 0, -100000000.0f), PxForceMode::eFORCE, true);
	//PxRigidBodyExt::addForceAtLocalPos(left->)
	//left->setAngularVelocity(PxVec3(10000, 0, 10000), true);
}
void moveRight(PxRigidDynamic* right) {
	//left->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
	//left->addTorque(PxVec3(0, 0, 1000000), PxForceMode::eFORCE, true);
	right->addForce(PxVec3(100000000.0f, 0, -100000000.0f), PxForceMode::eFORCE, true);
	//PxRigidBodyExt::addForceAtLocalPos(left->)
	//left->setAngularVelocity(PxVec3(10000, 0, 10000), true);
}

//创建关节
PxJoint* createMyJoint(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1) {
	PxRevoluteJoint* j = PxRevoluteJointCreate(*gPhysics, a0, t0, a1, t1);
	j->setProjectionLinearTolerance(0.1f);
	j->setConstraintFlag(PxConstraintFlag::ePROJECTION, true);
	return j;
}
PxJoint* createDampedD61(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1)
{
	PxD6Joint* j = PxD6JointCreate(*gPhysics, a0, t0, a1, t1);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
	//j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
	//j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
	//j->setLinearLimit(PxJointLinearLimit(1.0f, 0.1f));
	j->setSwingLimit(PxJointLimitCone(PxPi/6, PxPi/6, 1.0f));

	j->setProjectionLinearTolerance(0.1f);
	j->setConstraintFlag(PxConstraintFlag::ePROJECTION, true);
	j->setDrive(PxD6Drive::eSLERP, PxD6JointDrive(0, 1000, FLT_MAX, true));
	return j;
}
PxJoint* createDampedD62(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1)
{
	PxD6Joint* j = PxD6JointCreate(*gPhysics, a0, t0, a1, t1);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
	//j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
	//j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
	//j->setLinearLimit(PxJointLinearLimit(1.0f, 0.1f));
	j->setSwingLimit(PxJointLimitCone(PxPi/6, PxPi/6, 1.0f));
	j->setProjectionLinearTolerance(0.1f);
	j->setConstraintFlag(PxConstraintFlag::ePROJECTION, true);
	j->setDrive(PxD6Drive::eSLERP, PxD6JointDrive(0, 1000, FLT_MAX, true));
	return j;
}


bool isBall = false;
//发射小球
PxRigidDynamic* createBall(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity = PxVec3(100))
{
	dynamicBall = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
	dynamicBall->setAngularDamping(0.5f);
	dynamicBall->setLinearVelocity(velocity);
	dynamicBall->setRigidDynamicLockFlags(PxRigidDynamicLockFlag::eLOCK_LINEAR_Y);
	dynamicBall->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);

	//dynamicBall->setMass(1.0f);
	//dynamic->addForce(PxVec3(1, 0, 0),physx::PxForceMode::eFORCE , true);
	//dynamic->setAngularVelocity(velocity);
	PxShape* dynamicBallShape = PxRigidActorExt::createExclusiveShape(*dynamicBall, geometry, *gMaterial);
	dynamicBallShape->setSimulationFilterData(collisionGroupBall);//小球物碰撞标识
	gScene->addActor(*dynamicBall);
	isBall = true;
	
	return dynamicBall;
}

void removeBall() {
	gScene->removeActor(*dynamicBall);
	isBall = false;
}

void create_static(PxVec3 verts[], PxU32 size, PxVec3 globalpos) {
	gCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation,
		PxCookingParams(PxTolerancesScale()));
	PxRigidStatic* static_ptr = gPhysics->createRigidStatic(PxTransform(globalpos));
	PxConvexMesh* mesh = createConvexMesh(verts, size, *gPhysics, *gCooking);
	PxShape* shape = PxRigidActorExt::createExclusiveShape(*static_ptr, PxConvexMeshGeometry(mesh), *gMaterial);
	//shape->setSimulationFilterData(PxFilterData(COLLISION_FLAG_DRIVABLE_OBSTACLE,
	//	COLLISION_FLAG_DRIVABLE_OBSTACLE_AGAINST, PxPairFlag::eMODIFY_CONTACTS, 0));
	PxFilterData qryFilterData;
	//setupDrivableSurface(qryFilterData);
	shape->setQueryFilterData(qryFilterData);
	gScene->addActor(*static_ptr);
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
	sceneDesc.gravity = PxVec3(0.0f, 0.0f, 1.0f);

	sceneDesc.flags |= PxSceneFlag::eENABLE_CCD;
	sceneDesc.filterShader = ballFilterShader;

	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher	= gDispatcher;

	gScene = gPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.98f, 0.98f);


	//创建四周围墙
	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
	gScene->addActor(*groundPlane);
	PxShape* wallShape1 = gPhysics->createShape(PxBoxGeometry(100.0f,5.0f,1.5f), *gMaterial);
	wallShape1->setSimulationFilterData(collisionGroupSouthWall);//障碍物碰撞标识
	PxRigidStatic* southWall = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.0f, 5.0f, 200.0f)), *wallShape1);
	PxRigidStatic* northWall = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.0f, 5.0f, -200.0f)), *wallShape1);
	PxShape* wallShape2 = gPhysics->createShape(PxBoxGeometry(1.5f, 5.0f, 200.0f), *gMaterial);
	PxRigidStatic* eastWall = PxCreateStatic(*gPhysics, PxTransform(PxVec3(100.0f, 5.0f, 0.0f)), *wallShape2);
	PxRigidStatic* westWall = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-100.0f, 5.0f, 0.0f)), *wallShape2);
	gScene->addActor(*southWall);
	gScene->addActor(*westWall);
	gScene->addActor(*eastWall);
	gScene->addActor(*northWall);



	create_static(test_barrier, 12, PxVec3(-10, 0, 90));

	//小墙做阻隔
	PxShape* wallShape3 = gPhysics->createShape(PxBoxGeometry(1.0f, 5.0f, 160.0f), *gMaterial);
	PxRigidStatic* smallWall = PxCreateStatic(*gPhysics, PxTransform(PxVec3(90.0f, 5.0f, 40.0f)), *wallShape3);
	gScene->addActor(*smallWall);

	//PxShape* shape4 = gPhysics->createShape(PxBoxGeometry(1.0f, 10.0f, 100.0f), *gMaterial);
	//PxRigidStatic* leftStick = PxCreateStatic(*gPhysics, PxTransform(PxVec3(100.0f, 10.0f, 0.0f)), *shape2);

	//PxRigidStatic* rightStick = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-100.0f, 10.0f, 0.0f)), *shape2);


	//添加小球
	//PxRigidDynamic* ball = createDynamic(PxTransform(PxVec3(150.0f, 2.0f, 180.0f)), PxSphereGeometry(4.0f), PxVec3(0.0f, 0.0f, 0.0f));
	//ball->setRigidDynamicLockFlags( PxRigidDynamicLockFlag::eLOCK_LINEAR_Y);
	//ball->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
	//ball->addForce(PxVec3(0, 0, -10000000.0f), PxForceMode::eFORCE, true);
	//gScene->addActor(*ball);

	//左上方阻隔
	PxShape* TopLeftWall = gPhysics->createShape(PxBoxGeometry(55.0f, 5.0f, 2.5f), *gMaterial);
	PxTransform relativePose3(PxQuat(PxHalfPi*0.33, PxVec3(0, 1, 0)));
	TopLeftWall->setLocalPose(relativePose3);
	PxRigidStatic* stick6 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-52.5f, 5.0f, -175.0f)), *TopLeftWall);
	gScene->addActor(*stick6);

	//右上方阻隔
	PxShape* TopRightWall = gPhysics->createShape(PxBoxGeometry(50.0f, 5.0f, 2.5f), *gMaterial);
	PxTransform relativePose4(PxQuat(-0.33*PxHalfPi, PxVec3(0, 1, 0)));
	TopRightWall->setLocalPose(relativePose4);
	PxRigidStatic* stick7 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(55.0f, 5.0f, -175.0f)), *TopRightWall);
	gScene->addActor(*stick7);



	//胶囊障碍
	PxShape* capsuleShape = gPhysics->createShape(PxCapsuleGeometry(10.0f, 20.0f), *gMaterial);
	//PxRigidStatic* stick0 = gPhysics->createRigidStatic(PxTransform(PxVec3(10.0f, 0.0f, -70.0f)));
	PxTransform relativePose(PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
	capsuleShape->setLocalPose(relativePose);
	capsuleShape->setSimulationFilterData(collisionGroupObstacle);//障碍物碰撞标识

	//其他障碍物
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
	//PxShape* boxShape = gPhysics->createShape(PxBoxGeometry(10.0f, 10.0f, 10.0f), *gMaterial);
	//PxRigidStatic* box = PxCreateStatic(*gPhysics, PxTransform(PxVec3(40.0f, 10.0f, 50.0f)), *boxShape);
	//gScene->addActor(*box);
	//PxRigidStatic* box1 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-40.0f, 10.0f, 50.0f)), *boxShape);
	//gScene->addActor(*box1);
	PxShape* boxShape1 = gPhysics->createShape(PxBoxGeometry(10.0f, 10.0f, 10.0f), *gMaterial);
	PxTransform relativePose5(PxQuat(PxHalfPi*0.5, PxVec3(0, 1, 0)));
	boxShape1->setLocalPose(relativePose5);
	PxRigidStatic* box2 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-50.0f, 10.0f, -20.0f)), *boxShape1);
	gScene->addActor(*box2);
	PxRigidStatic* box3 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(50.0f, 10.0f, -20.0f)), *boxShape1);
	gScene->addActor(*box3);
	
	
	//右边摆臂
	//PxShape* rightHandWall = gPhysics->createShape(PxBoxGeometry(50.0f, 10.0f, 1.0f), *gMaterial);
	PxTransform relativePose1(PxQuat(PxHalfPi*0.33, PxVec3(0, 1, 0)));
	//rightHandWall->setLocalPose(relativePose1);
	//PxRigidStatic* stick4 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(45.0f, 10.0f, 120.0f)), *rightHandWall);
	//gScene->addActor(*stick4);
	//createChain(PxTransform(PxVec3(90.0f, 30.0f, 95.0f)), 2, PxBoxGeometry(25.0f, 10.0f, 1.0f), 49.0f, createDampedD6);
	PxVec3 offset(20.0f, 0, 0);
	PxTransform localTm(offset);

	//尝试创建右边摆臂
	PxShape* rightHandWall1 = gPhysics->createShape(PxBoxGeometry(35.0f, 5.0f, 2.5f), *gMaterial);
	//PxTransform relativePose1(PxQuat(PxHalfPi*0.33, PxVec3(0, 1, 0)));
	rightHandWall1->setLocalPose(relativePose1);


	PxRigidStatic* rightStaticStick = PxCreateStatic(*gPhysics, PxTransform(PxVec3(58.0f, 10.0f, 110.0f)), *rightHandWall1);
	current = PxCreateDynamic(*gPhysics, PxTransform(PxVec3(90.0f, 30.0f, 95.0f))*localTm, PxBoxGeometry(16.0f, 10.0f, 1.0f), *gMaterial, 1.0f);
	(*createDampedD62)(rightStaticStick, PxTransform(PxVec3(-27.0f, 0.0f, 16.5f)), current, PxTransform(offset));
	//current->setMass(30.0f);
	current->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
	gScene->addActor(*rightStaticStick);
	gScene->addActor(*current);
	

	//尝试创建左边摆臂
	//PxShape* leftHandWall = gPhysics->createShape(PxBoxGeometry(50.0f, 10.0f, 1.0f), *gMaterial);
	PxTransform relativePose2(PxQuat(-0.33*PxHalfPi, PxVec3(0, 1, 0)));
	//leftHandWall->setLocalPose(relativePose2);
	//PxRigidStatic* stick5 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-55.0f, 10.0f, 120.0f)), *leftHandWall);
	//gScene->addActor(*stick5);
	//stick->setRigidDynamicLockFlags(PxRigidDynamicLockFlag::eLOCK_LINEAR_Y);


	//尝试创建左边摆臂
	PxShape* leftHandWall1 = gPhysics->createShape(PxBoxGeometry(35.0f, 5.0f, 2.5f), *gMaterial);
	//PxTransform relativePose1(PxQuat(PxHalfPi*0.33, PxVec3(0, 1, 0)));
	leftHandWall1->setLocalPose(relativePose2);

	PxRigidStatic* leftStaticStick = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-70.0f, 10.0f, 110.0f)), *leftHandWall1);
	current1 = PxCreateDynamic(*gPhysics, PxTransform(PxVec3(90.0f, 30.0f, 95.0f))*localTm, PxBoxGeometry(16.0f, 10.0f, 1.0f), *gMaterial, 1.0f);
	//current1->setMass(30.0f);


	(*createDampedD61)(leftStaticStick, PxTransform(PxVec3(27.0f, 0.0f, 16.5f)), current1, PxTransform(-offset));
	current1->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
	gScene->addActor(*leftStaticStick);
	gScene->addActor(*current1);



	//PxShape* triShape = gPhysics->createShape(PxTriangleGeometry(10.0f,10.f,10.0f),*gMaterial);
	//PxRigidStatic* tri1 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.0f, 0.0f, 0.0f)), *triShape);
	//for(PxU32 i=0;i<5;i++)
	//createStack(PxTransform(PxVec3(0,0,stackZ-=10.0f)), 10, 2.0f);

	//发射小球
	if(!interactive)
		createDynamic(PxTransform(PxVec3(0,40,100)), PxSphereGeometry(10), PxVec3(0,0,0.1f));


}

//创建地图
void createMap() {

}

void stepPhysics(bool interactive)
{
	PX_UNUSED(interactive);
	gContactPositions.clear();
	gContactImpulses.clear();

	gScene->simulate(1.0f/60.0f);
	gScene->fetchResults(true);




	
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
	case 'T':	if (!isBall) 
	{ 
		createBall(PxTransform(PxVec3(95.0f, 2.0f, 184.0f)), PxSphereGeometry(3.55f), PxVec3(0, 0, -100)); 
		
	}
				break;
	case 'Q':   moveLeft(current1); break;//左摆臂
	case 'E':   moveRight(current); break;//右摆臂
	case 'I':   increaseScore(); break;
	case 'U':   removeBall(); break;
			/***default:
		current->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, false);
		current->setAngularVelocity(PxVec3(0, 0, -10000), true);
		current1->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, false);
		current1->setAngularVelocity(PxVec3(0, 0, -10000), true);
		***/

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
