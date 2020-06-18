

// 测试123
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
	if(key==27)
		exit(0);

	if(!sCamera->handleKey(key, x, y))
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
	Snippets::renderText(5, 96, "Press T to launch a ball.",25);
	Snippets::renderText(5, 90, "Press Q/E to hit the ball.", 25);
	Snippets::renderText(5, 84, "Your score:", 12);

	//在屏幕上打印分数
	string scoreString = to_string(score);
	char c[100];
	strcpy(c, scoreString.data());
	Snippets::renderText(30, 84, c, 10);

	PxScene* scene;
	PxGetPhysics().getScenes(&scene,1);
	PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
	if(nbActors)
	{
		std::vector<PxRigidActor*> actors(nbActors);
		scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
		Snippets::renderActors(&actors[0], static_cast<PxU32>(1), true,PxVec3(1.0f, 1.0f, 1.0f));
		Snippets::renderActors(&actors[1], static_cast<PxU32>(1), true, PxVec3(1.0f, 1.0f, 1.0f));
		Snippets::renderActors(&actors[2], static_cast<PxU32>(1), true, PxVec3(1.0f, 1.0f, 1.0f));
		Snippets::renderActors(&actors[3], static_cast<PxU32>(1), true, PxVec3(1.0f, 1.0f, 1.0f));
		Snippets::renderActors(&actors[4], static_cast<PxU32>(1), true, PxVec3(1.0f, 1.0f, 1.0f));
		Snippets::renderActors(&actors[5], static_cast<PxU32>(1), true, PxVec3(0.27f, 0.27f, 0.27f));
		Snippets::renderActors(&actors[6], static_cast<PxU32>(1), true, PxVec3(0.5f, 0.5f, 0.5f));
		Snippets::renderActors(&actors[7], static_cast<PxU32>(1), true, PxVec3(0.5f, 0.5f, 0.5f));
		Snippets::renderActors(&actors[8], static_cast<PxU32>(actors.size()-8), true, PxVec3(0.75f, 0.75f, 0.75f));
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

	sCamera = new Snippets::Camera(PxVec3(0.0f, 250.0f, 250.0f), PxVec3(0.0f,-6.f,-5.f));

	Snippets::setupDefaultWindow("PhysX Snippet HelloWorld");
	Snippets::setupDefaultRenderState();

	glutIdleFunc(idleCallback);
	glutDisplayFunc(renderCallback);
	glutKeyboardFunc(keyboardCallback);
	glutMouseFunc(mouseCallback);
	glutMotionFunc(motionCallback);
	motionCallback(0,0);

	atexit(exitCallback);

	initPhysics(true);
	glutMainLoop();
}
#endif

