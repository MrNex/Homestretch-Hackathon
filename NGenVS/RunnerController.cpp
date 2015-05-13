#include "RunnerController.h"

#include <stdlib.h>

#include "RenderingManager.h"

#include "GObject.h"

struct State_RunnerController_Members
{
	float acceleration;
	float maxVelocity;
	float angularVelocity;
	float jumpMag;
};

///
//Initializes a runner controller state
//
//Parameters:
//	state: A pointer to the state to initialize as a runner controller
//	acceleration: The acceleration of the runner
//	maxvelocity: The max speed of the runner
//	angularVelocity: the angular velocity of the runner
//	jumpMag: The magnitude of the impulse which will be applied when the player jumps
void State_RunnerController_Initialize(State* state, const float acceleration, const float maxVelocity, const float angularVelocity, const float jumpMag)
{
	struct State_RunnerController_Members* members = (struct State_RunnerController_Members*)malloc(sizeof(struct State_RunnerController_Members));
	state->members = (State_Members)members;

	members->acceleration = acceleration;
	members->angularVelocity = angularVelocity;
	members->maxVelocity = maxVelocity;
	members->jumpMag = jumpMag;

	state->State_Members_Free = State_RunnerController_Free;
	state->State_Update = State_RunnerController_Update;
}

///
//Frees members in a runner controller state
//
//Parameters:
//	state: A pointer to the state to free the members of
void State_RunnerController_Free(State* state)
{
	struct State_RunnerController_Members* members = (struct State_RunnerController_Members*)state->members;
	free(members);
}


///
//Updates the object attached to a runner controller
//This object will always accelerate forward while on a surface unless the velocity is already at the max velocity.
//
//Parameters:
//	obj: A pointer to the gameobject to update as a runner
//	state: A pointer to the runner controller state
void State_RunnerController_Update(GObject* obj, State* state)
{
	//If the object is colliding with something, allow it to accelerate
	if(obj->collider->currentCollisions->size > 0)
	{
		State_RunnerController_Accelerate(obj, state);
	}

	//Rotate runner
	State_RunnerController_Rotate(obj, state);

	//Grab the camera
	Camera* cam = RenderingManager_GetRenderingBuffer()->camera;

	// Set position of Camera to the body
	Camera_SetPosition(cam, obj->body->frame->position);
}

///
//Accelerates the runner controller
//
//PArameters:
//	obj: A pointer to the game object to accelerate
//	state: A pointer to rhe runner controller state which is accelerating this object
void State_RunnerController_Accelerate(GObject* obj, State* state)
{
	//Grab the state members
	struct State_RunnerController_Members* members = (struct State_RunnerController_Members*)state->members;

	//Grab the forward vector from the camera
	Camera* cam = RenderingManager_GetRenderingBuffer()->camera;

	Vector forward;
	Vector_INIT_ON_STACK(forward, 3);

	Matrix_SliceRow(&forward, cam->rotationMatrix, 2, 0, 3);

	//Project the forward vector onto the XY Plane
	Vector perp;
	Vector_INIT_ON_STACK(perp, 3);

	Vector_GetProjection(&perp, &forward, &Vector_E2);
	Vector_Decrement(&forward, &perp);

	//Scale the vector to the acceleration
	Vector_Normalize(&forward);
	Vector_Scale(&forward, -members->acceleration);



	//Only apply the impulse if the velocity is less than the max speed
	if(Vector_GetMag(obj->body->velocity) < members->maxVelocity)
	{
		//Apply the impulse
		RigidBody_ApplyImpulse(obj->body, &forward, &Vector_ZERO);
	}
}

///
//Rotates the runner controller
//	obj: A pointer to the game object to rotate
//	state: A pointer to the runner controller rotating the object
void State_RunnerController_Rotate(GObject* obj, State* state)
{

}