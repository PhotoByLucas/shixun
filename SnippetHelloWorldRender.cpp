// //爱谴责人士表示狗小组 陈思源、刘昊华、梁永辉、卢开书
#ifdef RENDER_SNIPPET

#include <vector>

#include "PxPhysicsAPI.h"
#include<iostream>
#include <string>
#include "score.h"

#include "../SnippetRender/SnippetRender.h"
#include "../SnippetRender/SnippetCamera.h"

using namespace physx;
using namespace std;

extern   int   maxScore;
extern   int   level;

extern void initPhysics(bool interactive);
extern void stepPhysics(bool interactive);
extern void cleanupPhysics(bool interactive);
extern void keyPress(unsigned char key, const PxTransform& camera);

namespace
{
	Snippets::Camera*	sCamera;

	void motionCallback(int x, int y)
	{
		sCamera->handleMotion(x, y);
	}

	void keyboardCallback(unsigned char key, int x, int y)
	{
		if (key == 27)
			exit(0);

		if (!sCamera->handleKey(key, x, y))
			keyPress(key, sCamera->getTransform());
	}

	void mouseCallback(int button, int state, int x, int y)
	{
		sCamera->handleMouse(button, state, x, y);
	}

	void idleCallback()
	{
		glutPostRedisplay();
	}

	void renderCallback()
	{
		stepPhysics(true);

		Snippets::startRender(sCamera->getEye(), sCamera->getDir());
		Snippets::renderText(5, 96, "Press T to launch a ball.", 25);
		Snippets::renderText(5, 90, "Press Q/E to hit the ball.", 25);
		Snippets::renderText(60, 96, "Your score:", 12);
		Snippets::renderText(70, 90, "Level:", 6);

		//在屏幕上打印分数
		string scoreString = to_string(score);
		char c[100];
		strcpy(c, scoreString.data());
		Snippets::renderText(84, 96, c, 4);

		//在屏幕上打印最大分数
		string maxscoreString = to_string(maxScore);
		char d[100];
		strcpy(d, maxscoreString.data());
		Snippets::renderText(91, 96, "/", 2);
		Snippets::renderText(93, 96, d, 5);

		//在屏幕上打印当前关卡
		string levelString = to_string(level);
		char e[100];
		strcpy(e, levelString.data());
		Snippets::renderText(86, 90, e, 3);

		//在屏幕上打印最大关卡
		Snippets::renderText(90, 90, "/", 2);
		Snippets::renderText(92, 90, "3", 2);

		PxScene* scene;
		PxGetPhysics().getScenes(&scene, 1);
		PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
		if (nbActors)
		{
			std::vector<PxRigidActor*> actors(nbActors);
			scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
			Snippets::renderActors(&actors[0], static_cast<PxU32>(1), true, PxVec3(1.0f, 1.0f, 1.0f));
			Snippets::renderActors(&actors[1], static_cast<PxU32>(1), true, PxVec3(1.0f, 1.0f, 1.0f));
			Snippets::renderActors(&actors[2], static_cast<PxU32>(1), true, PxVec3(1.0f, 1.0f, 1.0f));
			Snippets::renderActors(&actors[3], static_cast<PxU32>(1), true, PxVec3(1.0f, 1.0f, 1.0f));
			Snippets::renderActors(&actors[4], static_cast<PxU32>(1), true, PxVec3(1.0f, 1.0f, 1.0f));
			Snippets::renderActors(&actors[5], static_cast<PxU32>(1), true, PxVec3(0.5f, 0.5f, 0.5f));
			Snippets::renderActors(&actors[6], static_cast<PxU32>(1), true, PxVec3(0.5f, 0.5f, 0.5f));
			Snippets::renderActors(&actors[7], static_cast<PxU32>(1), true, PxVec3(0.5f, 0.5f, 0.5f));
			Snippets::renderActors(&actors[8], static_cast<PxU32>(actors.size() - 8), true, PxVec3(0.75f, 0.75f, 0.75f));
		}



		Snippets::finishRender();
	}

	void exitCallback(void)
	{
		delete sCamera;
		cleanupPhysics(true);
	}
}

void renderLoop()
{

	sCamera = new Snippets::Camera(PxVec3(0.0f, 250.0f, 250.0f), PxVec3(0.0f, -6.f, -5.f));

	Snippets::setupDefaultWindow("2022三维弹球");
	Snippets::setupDefaultRenderState();

	glutIdleFunc(idleCallback);
	glutDisplayFunc(renderCallback);
	glutKeyboardFunc(keyboardCallback);
	glutMouseFunc(mouseCallback);
	//glutMotionFunc(motionCallback);
	motionCallback(0, 0);

	atexit(exitCallback);

	initPhysics(true);
	glutMainLoop();
}
#endif