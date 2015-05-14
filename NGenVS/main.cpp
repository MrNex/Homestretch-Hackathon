///
//NGenVS is a Visual Studio port of the (currently unlicensed :( )
// NGen which can be found at www.github.com/MrNex/NGen
//
// Author: Nicholas Gallagher

//Enable CUDA Acceleration
//#define ENABLE_CUDA

#include <stdlib.h>
#include <stdio.h>



#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#endif

#include <GL\glew.h>
#include <GL\freeglut.h>


#include "InputManager.h"
#include "RenderingManager.h"
#include "AssetManager.h"
#include "ObjectManager.h"
#include "TimeManager.h"
#include "PhysicsManager.h"
#include "CollisionManager.h"

#include "ScoreState.h"
#include "ResetState.h"
#include "SpringState.h"
#include "ForceState.h"

#include "Matrix.h"

//#include "FirstPersonCameraState.h"
//#include "RotateState.h"
//#include "RevolutionState.h"
//#include "RotateCoordinateAxisState.h"
//#include "SpringState.h"
//#include "MeshSwapState.h"
//#include "MeshSpringState.h"
#include "CharacterController.h"
#include "RunnerController.h"

#include "LinkedList.h"
#include "DynamicArray.h"

#include "Generator.h"

#ifdef ENABLE_CUDA
#include <cuda.h>
#include <cuda_runtime.h>
#include "AcceleratedVector.h"
#endif

long timer;
unsigned char keyTrigger;

GObject* obj;

///
//Checks for any OpenGL errors and prints error code if found
void CheckGLErrors(void)
{
	//Get the openGL error code
	int error = glGetError();

	//If the error code signifies an error has occurred
	if (error != GL_NO_ERROR)
	{
		//Print the error code
		printf("Error: %d\n", error);
	}
}


///
//Adds a platform at the specified location with the specified scale
void AddPlatform(float x, float y, float z, float sx, float sy, float sz)
{
	GObject* obj = GObject_Allocate();
	GObject_Initialize(obj);

	obj->mesh = AssetManager_LookupMesh("Cube");
	obj->texture = AssetManager_LookupTexture("Test");

	obj->collider = Collider_Allocate();
	AABBCollider_Initialize(obj->collider, 2.0f, 2.0f, 2.0f, &Vector_ZERO);

	Vector transform;
	Vector_INIT_ON_STACK(transform, 3);
	transform.components[0] = x;
	transform.components[1] = y;
	transform.components[2] = z;
	GObject_Translate(obj, &transform);

	transform.components[0] = sx;
	transform.components[1] = sy;
	transform.components[2] = sz;
	GObject_Scale(obj, &transform);
	
	ObjectManager_AddObject(obj);

}


///
//Initializes the scene within the engine,
//Must be done after all vital engine components are initialized.
//This is all components excluding the TimeManager.
void InitializeScene(void)
{
	//Create the camera
	GObject* cam = GObject_Allocate();
	GObject_Initialize(cam);

	//Add collider & rigidbody
	cam->collider = Collider_Allocate();
	AABBCollider_Initialize(cam->collider, 3.0f, 3.0f, 3.0f, &Vector_ZERO);

	cam->body = RigidBody_Allocate();
	RigidBody_Initialize(cam->body, cam->frameOfReference, 1.0f);
	cam->body->coefficientOfRestitution = 0.0f;
	cam->body->dynamicFriction = 0.1f;

	//Attach character controller state
	State* state = State_Allocate();
	//State_CharacterController_Initialize(state, 0.01f, 0.005f, 5.0f, 1.0f);
	State_RunnerController_Initialize(state, 3.0f, 30.0f, 0.005f, 6.0f);
	GObject_AddState(cam, state);

	ObjectManager_AddObject(cam);

	//Create floor
	AddPlatform(0.0f, -10.0f, 0.0f, 100.0f, 1.0f, 300.0f);
	AddPlatform(0.0f, -5.0f, -300.0f, 100.0f, 1.0f, 30.0f);
	AddPlatform(-5.0f, 5.0f, -200.0f, 10.0f, 10.0f, 30.0f);
	
	//AddPlatform();
	//AddPlatform(45.0f, -5.0f, 0.0f, 10.0f, 0.1f, 50.0f);
	//AddPlatform(20.0f, 0.0f, 45.0f, 20.0f, 0.1f, 10.0f);
	//AddPlatform(0,0,0,100,100,100);

	//Set gravity
	Vector* gravity = Vector_Allocate();
	Vector_Initialize(gravity, 3);
	gravity->components[1] = -9.81f;
	
	PhysicsManager_AddGlobalAcceleration(gravity);
}

///
//Initializes all engine components
void Init(void)
{

	//Initialize managers
	InputManager_Initialize();
	RenderingManager_Initialize();
	AssetManager_Initialize();
	ObjectManager_Initialize();
	CollisionManager_Initialize();
	PhysicsManager_Initialize();

	//Load assets
	AssetManager_LoadAssets();


	//Cuda Testing
#ifdef ENABLE_CUDA
	//CUDA Test
	//Create Host variables
	int numVectors = 5;
	int vectorDim = 3;

	Vector* dotProd = Vector_Allocate();
	Vector_Initialize(dotProd, numVectors);

	Vector* scaledDotProd = Vector_Allocate();
	Vector_Initialize(scaledDotProd, numVectors);

	Vector* dests = Vector_Allocate();
	Vector_Initialize(dests, vectorDim);


	Vector** srcs = (Vector**)malloc(sizeof(Vector*)* numVectors);
	Vector** srcs2 = (Vector**)malloc(sizeof(Vector*)* numVectors);
	for (int i = 0; i < numVectors; i++)
	{


		srcs[i] = Vector_Allocate();
		srcs2[i] = Vector_Allocate();

		Vector_Initialize(srcs[i], vectorDim);
		Vector_Initialize(srcs2[i], vectorDim);

		//Set vectors value to 1
		for (int j = 0; j < vectorDim; j++)
		{	
			srcs[i]->components[j] = j;
			srcs2[i]->components[j] = 10;
		}

	}

	//Create device variables
	AcceleratedVector* aSrcs = AcceleratedVector_Allocate();
	AcceleratedVector* aSrcs2 = AcceleratedVector_Allocate();
	AcceleratedVector* aDests = AcceleratedVector_Allocate();
	AcceleratedVector* aDotProd = AcceleratedVector_Allocate();
	AcceleratedVector* aScaledDotProd = AcceleratedVector_Allocate();


	AcceleratedVector_Initialize(aSrcs, numVectors * vectorDim);
	AcceleratedVector_Initialize(aSrcs2, numVectors * vectorDim);
	AcceleratedVector_Initialize(aDests, vectorDim);
	AcceleratedVector_Initialize(aDotProd, numVectors);
	AcceleratedVector_Initialize(aScaledDotProd, numVectors);


	//Copy memory from host to device
	AcceleratedVector_CopyVector(aDests, dests);
	AcceleratedVector_CopyVector(aDotProd, dotProd);
	AcceleratedVector_CopyVector(aScaledDotProd, scaledDotProd);

	//Copy a metric fuckton of memory from the host to the device
	AcceleratedVector_CopyVectors(aSrcs, (const Vector**) srcs, vectorDim, numVectors);
	AcceleratedVector_CopyVectors(aSrcs2, (const Vector**) srcs2, vectorDim, numVectors);

	float scalar = 0.0625f;

	float* d_mag;
	float mag = 0.0f;
	cudaMalloc((void**)&d_mag, sizeof(float)* 1);

	//Launch kernel on GPU
	AcceleratedVector_LaunchAddAll(aDests->d_components, aSrcs->d_components, vectorDim, numVectors);
	AcceleratedVector_LaunchMagnitude(d_mag, aDests->d_components, vectorDim);
	AcceleratedVector_LaunchDotProductAll(aDotProd->d_components, aSrcs->d_components, aSrcs2->d_components, vectorDim, numVectors);
	AcceleratedVector_LaunchGetNormalize(aScaledDotProd->d_components, aDotProd->d_components, aDotProd->dimension);
	AcceleratedVector_LaunchProjectAll(aSrcs->d_components, aSrcs2->d_components, vectorDim, numVectors);

	//Copy memory from device to host	
	AcceleratedVector_PasteVector(dests, aDests);
	AcceleratedVector_PasteVector(dotProd, aDotProd);
	AcceleratedVector_PasteVector(scaledDotProd, aScaledDotProd);
	AcceleratedVector_PasteVectors(srcs, aSrcs, vectorDim, numVectors);
	AcceleratedVector_PasteVectors(srcs2, aSrcs2, vectorDim, numVectors);

	cudaMemcpy(&mag, d_mag, sizeof(float)* 1, cudaMemcpyDeviceToHost);

	//Print results
	Vector_Print(dests);
	Vector_Print(dotProd);
	Vector_Print(scaledDotProd);

	for (int i = 0; i < numVectors; i++)
	{
		Vector_Print(srcs[i]);
	}

	//Free device memory
	AcceleratedVector_Free(aDests);
	AcceleratedVector_Free(aSrcs);
	AcceleratedVector_Free(aSrcs2);
	AcceleratedVector_Free(aDotProd);
	AcceleratedVector_Free(aScaledDotProd);

	cudaFree(d_mag);

	//Free host memory

	for (int i = 0; i < numVectors; i++)
	{
		Vector_Free(srcs[i]);
		Vector_Free(srcs2[i]);
	}

	Vector_Free(dotProd);
	Vector_Free(dests);
	Vector_Free(scaledDotProd);

#endif

	//Initialize the scene
	InitializeScene();

	CheckGLErrors();

	//Time manager must always be initialized last
	TimeManager_Initialize();
}

void CalculateOctTreeCollisions(OctTree_Node* node)
{
	if(node->children != NULL)
	{
		for(int i = 0; i < 8; i++)
		{
			CalculateOctTreeCollisions(node->children+i);
		}
	}
	else
	{
		if(node->data->size != 0)
		{
			LinkedList* collisions = CollisionManager_UpdateArray((GObject**)node->data->data, node->data->size);
			PhysicsManager_ResolveCollisions(collisions);
		}
	}
}


///
//Updates engine
//
void Update(void)
{

	//Update time manager
	TimeManager_Update();

	/*
	long  dt = TimeManager_GetTimeBuffer().deltaTime->QuadPart;
	timer += dt;
	if (timer >= 100000)
	{
		printf("dt:\t%d\tmicroseconds\n", dt);
		timer = 0;

	}
	*/

	//Update objects.
	ObjectManager_Update();

	//Feature in development
	if(InputManager_IsKeyDown('g'))
	{
		TimeManager_SetTimeScale(0.0f);
	}
	if(InputManager_IsKeyDown('t'))
	{
		TimeManager_SetTimeScale(1.0f);
	}
	if (InputManager_IsKeyDown('r') || InputManager_IsKeyDown('y') || InputManager_IsKeyDown('o') || InputManager_IsKeyDown('p'))
	{
		if (keyTrigger == 0)
		{
			if (InputManager_IsKeyDown('r'))
			{
				TimeManager_ScaleTimeScale(0.9f);
			}
			else if (InputManager_IsKeyDown('y'))
			{
				TimeManager_ScaleTimeScale(1.1f);
			}
			else if (InputManager_IsKeyDown('o'))
			{
				RenderingManager_GetRenderingBuffer()->debugOctTree = 0;
			}
			else if (InputManager_IsKeyDown('p'))
			{
				RenderingManager_GetRenderingBuffer()->debugOctTree = 1;
			}
		}
		keyTrigger = 1;
	}
	else
	{
		keyTrigger = 0;
	}




	PhysicsManager_Update(ObjectManager_GetObjectBuffer().gameObjects);

	//Update the oct tree
	ObjectManager_UpdateOctTree();

	//LinkedList* collisions = CollisionManager_UpdateList(ObjectManager_GetObjectBuffer().gameObjects);

	//OctTree_Node* octTreeRoot = ObjectManager_GetObjectBuffer().octTree->root;
	//CalculateOctTreeCollisions(octTreeRoot);

	LinkedList* collisions = CollisionManager_UpdateOctTree(ObjectManager_GetObjectBuffer().octTree);


	//Pass collisions to physics manager to be resolved
	PhysicsManager_ResolveCollisions(collisions);

	//Update input
	InputManager_Update();

	CheckGLErrors();

}

void DrawLoop(int val)
{
	glutPostRedisplay();
	glutTimerFunc(16, DrawLoop, 0);


}

///
//Draws the current state of the engine
void Draw(void)
{
	RenderingManager_Render(ObjectManager_GetObjectBuffer().gameObjects);
}


///
//Program will begin here
//
//Parameters:
//	int argc - Number of arguments passed from cmd line
//	char* argv[] - Array of C strings of arguments passed from cmd line
int main(int argc, char* argv[])
{
	//Initialize glut
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA);
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(600, 0);
	glutInitContextVersion(4, 3);
	glutInitContextProfile(GLUT_CORE_PROFILE);

	//Window creation
	int win = glutCreateWindow("NGenVS V3.8: Shooting Gallery");

	glewExperimental = GL_TRUE;
	if (glewInit() != GLEW_OK) { return -1; }


	//Check for errors
	CheckGLErrors();

	///
	//Set up callback registration
	//
	glutIdleFunc(Update);
	glutTimerFunc(16, DrawLoop, 0);
	glutDisplayFunc(Draw);


	//Callback registration for Input
	glutPassiveMotionFunc(InputManager_OnMouseMove);
	glutMotionFunc(InputManager_OnMouseDrag);
	glutMouseFunc(InputManager_OnMouseClick);
	glutKeyboardFunc(InputManager_OnKeyPress);
	glutKeyboardUpFunc(InputManager_OnKeyRelease);

	//Initialize engine
	Init();

	//Start the main loop
	glutMainLoop();

	//When the loop is over, release all memory
	glutDestroyWindow(win);

	InputManager_Free();
	RenderingManager_Free();
	ObjectManager_Free();
	AssetManager_Free();
	CollisionManager_Free();
	PhysicsManager_Free();
	TimeManager_Free();

	return 0;
}