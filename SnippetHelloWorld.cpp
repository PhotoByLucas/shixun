//爱谴责人士表示狗小组 陈思源、刘昊华、梁永辉、卢开书

#include<Windows.h>
#include<mmsystem.h>
#pragma comment(lib,"winmm.lib")


#include <conio.h>
// 引用 Windows Multimedia API

#include <ctime>
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
#include <iostream>
#include <vector>

using namespace physx;
using namespace std;

PxCooking*				gCooking = NULL;
PxDefaultAllocator	gAllocator;
PxDefaultErrorCallback gErrorCallback;

PxFoundation*	gFoundation = NULL;
PxPhysics*	gPhysics = NULL;

PxDefaultCpuDispatcher*	gDispatcher = NULL;
PxScene*	gScene = NULL;

//球体，障碍材质
PxMaterial*	gMaterial = NULL;

//设置墙壁材质
PxMaterial*	wallMaterial = NULL;

//设置旋转棒材质
PxMaterial*	moveBoxMaterial = NULL;

//设置球材质
PxMaterial*	 ballMaterial = NULL;

PxPvd* gPvd = NULL;

PxRigidDynamic* dynamicBall = NULL;
PxRigidDynamic* random_ptr = NULL;
PxRigidStatic* plane;



std::vector<PxVec3> gContactPositions;
std::vector<PxVec3> gContactImpulses;



PxRigidDynamic* current = NULL;//左挡板
PxRigidDynamic* current1 = NULL;//右挡板

PxRigidStatic* stick1 = NULL;//地图1中胶囊1
PxRigidStatic* stick2 = NULL;//地图1中胶囊2
PxRigidStatic* stick3 = NULL;//地图1中胶囊3
PxRigidStatic* box = NULL; //地图1中正方体
PxRigidStatic* box1 = NULL;//地图1中正方体1
PxRigidStatic* box2 = NULL;//地图1中正方体2
PxRigidStatic* box3 = NULL; //地图1中正方体3
PxRigidDynamic* moveBox = NULL;//旋转棒
PxRigidDynamic* moveBox1 = NULL;//旋转棒1
PxRigidDynamic* moveBox2 = NULL;//旋转棒2
PxRigidStatic* static_ptr = NULL;//六边形
PxRigidStatic* static_ptr1 = NULL;//六边形
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
PxVec3 test_barrier1[] = {
	PxVec3(0,0,0),
	PxVec3(20,0,0),
	PxVec3(-10,0,sqrt(300)),
	PxVec3(0,0,sqrt(1200)),
	PxVec3(30,0,sqrt(300)),
	PxVec3(20,0,sqrt(1200)),

	PxVec3(0,10,0),
	PxVec3(20,10,0),
	PxVec3(-10,10,sqrt(300)),
	PxVec3(0,10,sqrt(1200)),
	PxVec3(30,10,sqrt(300)),
	PxVec3(20,10,sqrt(1200))
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
	pairFlags = PxPairFlag::eSOLVE_CONTACT;
	pairFlags |= PxPairFlag::eDETECT_DISCRETE_CONTACT;
	pairFlags |= PxPairFlag::eDETECT_CCD_CONTACT;
	if (PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
	{
		pairFlags |= PxPairFlag::eTRIGGER_DEFAULT;
		return PxFilterFlag::eDEFAULT;
	}
	// generate contacts for all that were not filtered above
	pairFlags |= PxPairFlag::eCONTACT_DEFAULT;

	// trigger the contact callback for pairs (A,B) where 
	// the filtermask of A contains the ID of B and vice versa.
	//if ((filterData0.word1 & filterData1.word1 == 0) && (filterData0.word2 & filterData1.word2 == 0) && (filterData0.word3 & filterData1.word3 == 1))//下面墙的检测
	//{
	//	gScene->removeActor(*dynamicBall);
	//	isBall = false;
	//	pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
	//}

	//if ((filterData0.word1 & filterData1.word1 == 0)&& (filterData0.word2 & filterData1.word2 ==1 )&& (filterData0.word3 & filterData1.word3 == 0))//胶囊体的检测
	//{
	//	score += 5;
	//	pairFlags|= PxPairFlag::eNOTIFY_TOUCH_FOUND;
	//}

	//if ((filterData0.word1 & filterData1.word1 == 0) && (filterData0.word2 & filterData1.word2 == 1) && (filterData0.word3 & filterData1.word3 == 1))//box的检测
	//{
	//	score += 2;
	//	pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
	//}

	//if ((filterData0.word1 & filterData1.word1 == 1) && (filterData0.word2 & filterData1.word2 == 0) && (filterData0.word3 & filterData1.word3 == 0))//moveBox的检测
	//{
	//	score += 7;
	//	pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
	//}

	if ((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
	{

		gScene->removeActor(*dynamicBall);
		isBall = false;
		pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
	}

	if ((filterData0.word0 & filterData1.word2) && (filterData1.word0 & filterData0.word2))
	{
		score += 1;
		pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
	}

	if ((filterData0.word0 & filterData1.word3) && (filterData1.word0 & filterData0.word3))
	{
		score += 2;
		pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
	}

	return PxFilterFlags();
}

//static const PxFilterData collisionGroupBall(1, 1, 1, 1);
//static const PxFilterData collisionGroupSouthWall(0, 0, 0, 1);
//static const PxFilterData collisionGroupCapsule(0, 0,1, 0);
//static const PxFilterData collisionGroupCapsule1(0, 0, 1, 0);
//static const PxFilterData collisionGroupBox(0 ,0 , 1 , 1);
//static const PxFilterData collisionGroupMoveBox(0,1, 0, 0);

static const PxFilterData collisionGroupBall(1, 1, 1, 1);
static const PxFilterData collisionGroupSouthWall(1, 1, 0, 0);
static const PxFilterData collisionGroupCapsule(1, 0, 1, 0);
static const PxFilterData collisionGroupBox(1, 0, 0, 1);
static const PxFilterData collisionGroupRandom(1, 0, 1, 1);


PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity = PxVec3(100))
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);

	//dynamic->setAngularVelocity(velocity);
	gScene->addActor(*dynamic);


	return dynamic;
}

int score = 0;

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

PxJoint* createDampedD61(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1)
{
	PxD6Joint* j = PxD6JointCreate(*gPhysics, a0, t0, a1, t1);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
	//j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
	//j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
	//j->setLinearLimit(PxJointLinearLimit(1.0f, 0.1f));
	j->setSwingLimit(PxJointLimitCone(PxPi / 6, PxPi / 6, 1.0f));

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
	j->setSwingLimit(PxJointLimitCone(PxPi / 6, PxPi / 6, 1.0f));
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

	PxShape* dynamicBallShape = PxRigidActorExt::createExclusiveShape(*dynamicBall, geometry, *ballMaterial);
	dynamicBallShape->setSimulationFilterData(collisionGroupBall);//小球碰撞标识
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
	static_ptr = gPhysics->createRigidStatic(PxTransform(globalpos));
	PxConvexMesh* mesh = createConvexMesh(verts, size, *gPhysics, *gCooking);
	PxShape* shape = PxRigidActorExt::createExclusiveShape(*static_ptr, PxConvexMeshGeometry(mesh), *gMaterial);
	//为了增加难度 mesh不加分
	gScene->addActor(*static_ptr);
}

bool isRandom = false;
void create_random() {
	PxVec3 random_pos;
	srand((int)time(0));  // 产生随机种子  把0换成NULL也行
	float f[3] = {};
	for (int i = 0; i < 3; i++)
	{
		f[i] = -30 + rand() % 60;
	}
	random_pos = PxVec3(f[0], 4.0f, f[2]);
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(4.0f, 4.0f, 4.0f), *gMaterial);
	shape->setSimulationFilterData(collisionGroupRandom);//随机点碰撞标识	
	random_ptr = gPhysics->createRigidDynamic(PxTransform(random_pos));
	random_ptr->attachShape(*shape);
	random_ptr->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
	gScene->addActor(*random_ptr);
	isRandom = true;
}
void removeRandom() {
	gScene->removeActor(*random_ptr);
	isRandom = false;
}


//花花地图初始化

void createMap1() {
	//胶囊障碍
	PxShape* capsuleShape = gPhysics->createShape(PxCapsuleGeometry(10.0f, 20.0f), *gMaterial);
	
	PxTransform relativePose(PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
	capsuleShape->setLocalPose(relativePose);
	capsuleShape->setSimulationFilterData(collisionGroupCapsule);//障碍物碰撞标识
	//其他障碍
	stick1 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.0f, 0.0f, -100.0f)), *capsuleShape);
	gScene->addActor(*stick1);
	stick2 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.0f, 0.0f, 50.0f)), *capsuleShape);
	gScene->addActor(*stick2);
	stick3 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.0f, 0.0f, -50.0f)), *capsuleShape);
	gScene->addActor(*stick3);



	PxShape* boxShape1 = gPhysics->createShape(PxBoxGeometry(10.0f, 10.0f, 10.0f), *gMaterial);
	PxTransform relativePose5(PxQuat(PxHalfPi*0.5, PxVec3(0, 1, 0)));
	boxShape1->setLocalPose(relativePose5);
	boxShape1->setSimulationFilterData(collisionGroupBox);//障碍物碰撞标识
	box2 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-50.0f, 10.0f, -20.0f)), *boxShape1);
	gScene->addActor(*box2);
	box3 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(50.0f, 10.0f, -20.0f)), *boxShape1);
	gScene->addActor(*box3);
}

void createMap2() {

	gScene->removeActor(*stick1);
	gScene->removeActor(*stick2);
	gScene->removeActor(*stick3);
	gScene->removeActor(*box2);
	gScene->removeActor(*box3);
	PxShape* capsuleShape = gPhysics->createShape(PxCapsuleGeometry(10.0f, 20.0f), *gMaterial);


	PxTransform relativePose(PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
	capsuleShape->setLocalPose(relativePose);
	capsuleShape->setSimulationFilterData(collisionGroupCapsule);//障碍物碰撞标识

	//其他障碍
	stick1 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(50.0f, 0.0f, 0.0f)), *capsuleShape);
	//stick->setRigidDynamicLockFlags(PxRigidDynamicLockFlag::eLOCK_LINEAR_Y);
	gScene->addActor(*stick1);
	//PxShape* boxShape = gPhysics->createShape(PxBoxGeometry(10.0f, 10.0f, 10.0f), *gMaterial);
	//box = PxCreateStatic(*gPhysics, PxTransform(PxVec3(40.0f, 10.0f, 50.0f)), *boxShape);
	//gScene->addActor(*box);
	//box1 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-40.0f, 10.0f, 50.0f)), *boxShape);
	//gScene->addActor(*box1);
	moveBox = createDynamic(PxTransform(PxVec3(40.0f, 4.1f, 50.0f)), PxBoxGeometry(4.0f, 4.0f, 20.0f), PxVec3(0.0f, 0.0f, 0.0f));

	moveBox->setAngularVelocity(PxVec3(0.0f, .5f, 0.0f));
	moveBox->setAngularDamping(0.f);
	moveBox->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
	//moveBox->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
	PxRigidBodyExt::updateMassAndInertia(*moveBox, 100000.0f);

	PxShape* dynamicMoveBox = PxRigidActorExt::createExclusiveShape(*moveBox, PxBoxGeometry(4.0f, 4.0f, 20.0f), *gMaterial);
	dynamicMoveBox->setSimulationFilterData(collisionGroupBox);//障碍物碰撞标识

	moveBox->setRigidDynamicLockFlags(
		PxRigidDynamicLockFlag::eLOCK_LINEAR_Y |
		PxRigidDynamicLockFlag::eLOCK_ANGULAR_X |
		PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z
	);

	moveBox1 = createDynamic(PxTransform(PxVec3(-70.0f, 4.1f, 0.0f)), PxBoxGeometry(4.0f, 4.0f, 20.0f), PxVec3(0.0f, 0.0f, 0.0f));
	moveBox1->setAngularVelocity(PxVec3(0.0f, 1.f, 0.0f));
	moveBox1->setAngularDamping(0.0f);
	moveBox1->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
	//moveBox->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
	PxRigidBodyExt::updateMassAndInertia(*moveBox1, 100000.0f);

	PxShape* dynamicMoveBox1 = PxRigidActorExt::createExclusiveShape(*moveBox1, PxBoxGeometry(4.0f, 4.0f, 20.0f), *moveBoxMaterial);
	dynamicMoveBox1->setSimulationFilterData(collisionGroupBox);//障碍物碰撞标识

	moveBox1->setRigidDynamicLockFlags(
		PxRigidDynamicLockFlag::eLOCK_LINEAR_Y |
		PxRigidDynamicLockFlag::eLOCK_ANGULAR_X |
		PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z
	);

	moveBox2 = createDynamic(PxTransform(PxVec3(40.0f, 4.1f, -100.0f)), PxBoxGeometry(4.0f, 4.0f, 20.0f), PxVec3(0.0f, 0.0f, 0.0f));
	moveBox2->setAngularVelocity(PxVec3(0.0f, .5f, 0.0f));
	moveBox2->setAngularDamping(0.f);
	moveBox2->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
	//moveBox->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
	PxRigidBodyExt::updateMassAndInertia(*moveBox2, 10000000.0f);

	PxShape* dynamicMoveBox2 = PxRigidActorExt::createExclusiveShape(*moveBox2, PxBoxGeometry(4.0f, 4.0f, 20.0f), *moveBoxMaterial);
	dynamicMoveBox2->setSimulationFilterData(collisionGroupBox);//障碍物碰撞标识

	moveBox2->setRigidDynamicLockFlags(
		PxRigidDynamicLockFlag::eLOCK_LINEAR_Y |
		PxRigidDynamicLockFlag::eLOCK_ANGULAR_X |
		PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z
	);


}
void createMap3() {
	gScene->removeActor(*moveBox);
	gScene->removeActor(*moveBox1);
	gScene->removeActor(*moveBox2);
	gScene->removeActor(*stick1);
	//create_static(test_barrier, 12, PxVec3(-10, 0, -50));

	create_static(test_barrier1, 12, PxVec3(45, 0, -50));
	create_static(test_barrier1, 12, PxVec3(-65, 0, -50));
	moveBox = createDynamic(PxTransform(PxVec3(-5.0f, 4.0f, 70.0f)), PxBoxGeometry(4.0f, 4.0f, 25.0f), PxVec3(0.0f, 0.0f, 0.0f));

	moveBox->setAngularVelocity(PxVec3(0.0f, .5f, 0.0f));
	moveBox->setAngularDamping(0.f);
	moveBox->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
	//moveBox->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
	PxRigidBodyExt::updateMassAndInertia(*moveBox, 100000.0f);

	PxShape* dynamicMoveBox = PxRigidActorExt::createExclusiveShape(*moveBox, PxBoxGeometry(4.0f, 4.0f, 20.0f), *moveBoxMaterial);
	dynamicMoveBox->setSimulationFilterData(collisionGroupBox);//障碍物碰撞标识

	moveBox->setRigidDynamicLockFlags(
		PxRigidDynamicLockFlag::eLOCK_LINEAR_Y |
		PxRigidDynamicLockFlag::eLOCK_ANGULAR_X |
		PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z
	);
	moveBox1 = createDynamic(PxTransform(PxVec3(-5.0f, 4.0f, -100.0f)), PxBoxGeometry(4.0f, 4.0f, 20.0f), PxVec3(0.0f, 0.0f, 0.0f));
	moveBox1->setAngularVelocity(PxVec3(0.0f, .5f, 0.0f));
	moveBox1->setAngularDamping(0.f);
	moveBox1->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
	//moveBox->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
	PxRigidBodyExt::updateMassAndInertia(*moveBox1, 100000.0f);
	moveBox1->setRigidDynamicLockFlags(
		PxRigidDynamicLockFlag::eLOCK_LINEAR_Y |
		PxRigidDynamicLockFlag::eLOCK_ANGULAR_X |
		PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z
	);
}
void initPhysics(bool interactive)
{
	
	//mciSendString(("open bgm.mp3 alias MUSIC"),NULL, 0, NULL);
	//mciSendString("play MUSIC repeat", NULL, 0, NULL);

	gFoundation = PxCreateFoundation(PX_FOUNDATION_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);
	PxInitExtensions(*gPhysics, gPvd);
	//PxU32 numCores = SnippetUtils::getNbPhysicalCores();

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, 0.0f, 1.0f);

	sceneDesc.flags |= PxSceneFlag::eENABLE_CCD;
	sceneDesc.filterShader = ballFilterShader;

	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = gDispatcher;

	gScene = gPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.0f, 1.0f);//所有障碍物材质，地板材质
	ballMaterial= gPhysics->createMaterial(0.0f, 0.98f, 0.4f);//球的材质
	wallMaterial = gPhysics->createMaterial(0.0f, 0.0f, 0.0f);//所有墙壁的材质与小球的材质
	moveBoxMaterial = gPhysics->createMaterial(0.5f, 0.0f, 0.0f);//静摩擦系数，动摩擦系数，弹性系数

	//创建地板
	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	gScene->addActor(*groundPlane);
	//创建四周围墙
	PxShape* wallShapeSpecial = gPhysics->createShape(PxBoxGeometry(100.0f, 5.0f, 1.5f), *gMaterial);
	wallShapeSpecial->setSimulationFilterData(collisionGroupSouthWall);//障碍物碰撞标识
	PxShape* wallShape1 = gPhysics->createShape(PxBoxGeometry(100.0f, 5.0f, 1.5f), *gMaterial);
	PxRigidStatic* southWall = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.0f, 5.0f, 200.0f)), *wallShapeSpecial);

	PxRigidStatic* northWall = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.0f, 5.0f, -200.0f)), *wallShape1);
	PxShape* wallShape2 = gPhysics->createShape(PxBoxGeometry(1.5f, 5.0f, 200.0f), *gMaterial);
	PxRigidStatic* eastWall = PxCreateStatic(*gPhysics, PxTransform(PxVec3(100.0f, 5.0f, 0.0f)), *wallShape2);
	PxRigidStatic* westWall = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-100.0f, 5.0f, 0.0f)), *wallShape2);
	gScene->addActor(*southWall);
	gScene->addActor(*westWall);
	gScene->addActor(*eastWall);
	gScene->addActor(*northWall);

	//小墙做阻隔

	PxShape* wallShape3 = gPhysics->createShape(PxBoxGeometry(1.0f, 5.0f, 160.0f), *gMaterial);
	PxRigidStatic* smallWall = PxCreateStatic(*gPhysics, PxTransform(PxVec3(90.0f, 5.0f, 40.0f)), *wallShape3);
	gScene->addActor(*smallWall);


	//创建上方左侧斜小墙

	PxShape* TopLeftWall = gPhysics->createShape(PxBoxGeometry(55.0f, 5.0f, 2.5f), *gMaterial);
	PxTransform relativePose3(PxQuat(PxHalfPi*0.33, PxVec3(0, 1, 0)));
	TopLeftWall->setLocalPose(relativePose3);
	PxRigidStatic* stick6 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-52.5f, 5.0f, -175.0f)), *TopLeftWall);
	gScene->addActor(*stick6);


	//创建上方右侧斜小墙
	PxShape* TopRightWall = gPhysics->createShape(PxBoxGeometry(50.0f, 5.0f, 2.5f), *gMaterial);
	PxTransform relativePose4(PxQuat(-0.33*PxHalfPi, PxVec3(0, 1, 0)));
	TopRightWall->setLocalPose(relativePose4);
	PxRigidStatic* stick7 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(55.0f, 5.0f, -175.0f)), *TopRightWall);
	gScene->addActor(*stick7);

	/***设计地图部分***/
	createMap1();




	PxTransform relativePose6(PxQuat(PxHalfPi, PxVec3(0, 1, 0)));
	PxShape* capsuleShape1 = gPhysics->createShape(PxCapsuleGeometry(10.0f, 10.0f), *gMaterial);
	capsuleShape1->setLocalPose(relativePose6);
	PxRigidStatic* capsule1 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(70.0f, 0.0f, 70.0f)), *capsuleShape1);
	//stick->setRigidDynamicLockFlags(PxRigidDynamicLockFlag::eLOCK_LINEAR_Y);
	gScene->addActor(*capsule1);
	PxRigidStatic* capsule2 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-80.0f, 0.0f, 70.0f)), *capsuleShape1);
	gScene->addActor(*capsule2);


	//右边摆臂

	//下方是地图固定部分
	//下方右边的阻隔

	//PxShape* rightHandWall = gPhysics->createShape(PxBoxGeometry(50.0f, 10.0f, 1.0f), *gMaterial);
	PxTransform relativePose1(PxQuat(PxHalfPi*0.33, PxVec3(0, 1, 0)));
	//rightHandWall->setLocalPose(relativePose1);
	//PxRigidStatic* stick4 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(45.0f, 10.0f, 120.0f)), *rightHandWall);
	//gScene->addActor(*stick4);
	//createChain(PxTransform(PxVec3(90.0f, 30.0f, 95.0f)), 2, PxBoxGeometry(25.0f, 10.0f, 1.0f), 49.0f, createDampedD6);
	PxVec3 offset(20.0f, 0, 0);
	PxTransform localTm(offset);


	//尝试创建右边摆臂
	PxShape* rightHandWall1 = gPhysics->createShape(PxBoxGeometry(35.0f, 7.0f, 2.5f), *wallMaterial);

	//PxTransform relativePose1(PxQuat(PxHalfPi*0.33, PxVec3(0, 1, 0)));
	rightHandWall1->setLocalPose(relativePose1);


	PxRigidStatic* rightStaticStick = PxCreateStatic(*gPhysics, PxTransform(PxVec3(58.0f, 7.0f, 110.0f)), *rightHandWall1);
	current = PxCreateDynamic(*gPhysics, PxTransform(PxVec3(90.0f, 30.0f, 95.0f))*localTm, PxBoxGeometry(16.0f, 7.0f, 1.0f), *wallMaterial, 1.0f);
	(*createDampedD62)(rightStaticStick, PxTransform(PxVec3(-27.0f, 0.0f, 16.5f)), current, PxTransform(offset));
	//current->setMass(30.0f);
	current->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
	gScene->addActor(*rightStaticStick);
	gScene->addActor(*current);



	//创建下方左侧斜小墙
	//PxShape* leftHandWall = gPhysics->createShape(PxBoxGeometry(50.0f, 10.0f, 1.0f), *gMaterial);
	PxTransform relativePose2(PxQuat(-0.33*PxHalfPi, PxVec3(0, 1, 0)));
	//leftHandWall->setLocalPose(relativePose2);
	//PxRigidStatic* stick5 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-55.0f, 10.0f, 120.0f)), *leftHandWall);
	//gScene->addActor(*stick5);
	//stick->setRigidDynamicLockFlags(PxRigidDynamicLockFlag::eLOCK_LINEAR_Y);



	//尝试创建左边摆臂
	PxShape* leftHandWall1 = gPhysics->createShape(PxBoxGeometry(35.0f, 7.0f, 2.5f), *wallMaterial);

	//PxTransform relativePose1(PxQuat(PxHalfPi*0.33, PxVec3(0, 1, 0)));
	leftHandWall1->setLocalPose(relativePose2);

	PxRigidStatic* leftStaticStick = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-68.0f, 7.0f, 110.0f)), *leftHandWall1);
	current1 = PxCreateDynamic(*gPhysics, PxTransform(PxVec3(90.0f, 30.0f, 95.0f))*localTm, PxBoxGeometry(16.0f, 7.0f, 1.0f), *wallMaterial, 1.0f);
	//current1->setMass(30.0f);


	(*createDampedD61)(leftStaticStick, PxTransform(PxVec3(27.0f, 0.0f, 16.5f)), current1, PxTransform(-offset));
	current1->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
	gScene->addActor(*leftStaticStick);
	gScene->addActor(*current1);


}
int maxScore = 50;
int level = 1;
int randomTime = 0;
void stepPhysics(bool interactive)
{
	PX_UNUSED(interactive);
	if (score >= 50&&level==1) {
		MessageBox(0, "Congratulations on reaching the points you need to enter the next level. \nYou will enter the next level", "Congratulations", 0);
		removeBall();
		createMap2();
		maxScore = 100;
		level = 2;
	}
	if (score >= 100&&level==2) {
		MessageBox(0, "Congratulations on reaching the points you need to enter the next level. \nYou will enter the final level", "Congratulations", 0);
		removeBall();
		createMap3();
		maxScore = 999;
		level = 3;
	}
	randomTime++;
	if (randomTime == 4000) {
		if (isRandom) {
			removeRandom();
		}
		create_random();
		randomTime = 0;
	}


	gContactPositions.clear();
	gContactImpulses.clear();

	gScene->simulate(1.0f / 60.0f);
	gScene->fetchResults(true);
	// 到达指定区域之后gScene->removeActor()小球


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
	switch (toupper(key))
	{
	//case 'B':	createStack(PxTransform(PxVec3(0, 0, stackZ -= 10.0f)), 10, 2.0f);						break;
	//case ' ':	createDynamic(camera, PxSphereGeometry(4.0f), camera.rotate(PxVec3(0, 0, -1)) * 200);	break;
	case 'T':	if (!isBall) { createBall(PxTransform(PxVec3(95.0f, 2.0f, 184.0f)), PxSphereGeometry(3.55f), PxVec3(0, 0, -300)); }
				break;
	case 'Q':   moveLeft(current1); break;//移动左挡板
	case 'E':   moveRight(current); break;//移动右挡板
	//case 'U':   removeBall(); break;
	//case '2':   createMap2(); break;
	//case '3':   createMap3(); break;

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
	for (PxU32 i = 0; i < frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}