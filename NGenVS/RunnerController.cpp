#include "RunnerController.h"

#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "RenderingManager.h"
#include "InputManager.h"
#include "CollisionManager.h"

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

		//If the user is pressing LMB
		if(InputManager_IsMouseButtonPressed(0))
		{
			State_RunnerController_Jump(obj, state);
		}
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
	if(Vector_GetMag(obj->body->velocity) - fabs(Vector_DotProduct(obj->body->velocity, &Vector_E2)) < members->maxVelocity)
	{
		//Apply the impulse
		RigidBody_ApplyImpulse(obj->body, &forward, &Vector_ZERO);
	}
	else
	{
		printf("Value:\t%f\n", Vector_GetMag(obj->body->velocity) - fabs(Vector_DotProduct(obj->body->velocity, &Vector_E2)));
	}
}

///
//Rotates the runner controller
//	obj: A pointer to the game object to rotate
//	state: A pointer to the runner controller rotating the object
void State_RunnerController_Rotate(GObject* obj, State* state)
{
	// create a camera object
	Camera* cam = RenderingManager_GetRenderingBuffer()->camera;

	//Grab the state members
	struct State_RunnerController_Members* members = (struct State_RunnerController_Members*)state->members;

	// if player's mouse is locked
	if(InputManager_GetInputBuffer().mouseLock)
	{

		int deltaMouseX = (InputManager_GetInputBuffer().mousePosition[0] - InputManager_GetInputBuffer().previousMousePosition[0]);
		int deltaMouseY = (InputManager_GetInputBuffer().mousePosition[1] - InputManager_GetInputBuffer().previousMousePosition[1]);

		Vector* axis = Vector_Allocate();
		Vector_Initialize(axis,3);

		if(deltaMouseX != 0)
		{
			axis->components[1] = 1.0f;
			// rotate the camera
			Camera_ChangeYaw(cam, members->angularVelocity * deltaMouseX);
			axis->components[1] = 0.0f;
		}

		if (deltaMouseY != 0)
		{
			Vector forwardVector;
			Vector_INIT_ON_STACK(forwardVector, 3);
			Matrix_SliceRow(&forwardVector, cam->rotationMatrix, 2, 0, 3);

			// Keep camera from overextending it's boundaries.
			if (deltaMouseY > 0)
			{
				if (Vector_DotProduct(&forwardVector, &Vector_E2) < 0.7f)
				{
					axis->components[0] = 1.0f;
					Camera_ChangePitch(cam, members->angularVelocity * deltaMouseY);
					axis->components[0] = 0.0f;
				}
			}
			else if (deltaMouseY < 0)
			{
				if (Vector_DotProduct(&forwardVector, &Vector_E2) > -0.7f)
				{
					axis->components[0] = 1.0f;
					Camera_ChangePitch(cam, members->angularVelocity * deltaMouseY);
					axis->components[0] = 0.0f;
				}
			}
		}
	}
}

///
//Allows the runner controller to jump if necessary conditions are met
//
//Parameters:
//	obj: A pointer to the object jumping
//	state: A pointer to the runner controller state which is jumping the object
void State_RunnerController_Jump(GObject* obj, State* state)
{
	//Check if the object is on the ground...
	unsigned char onGround = 0;

	//Loop through all current collisions
	LinkedList_Node* current = obj->collider->currentCollisions->head;
	Collision* currentCollision = NULL;
	while(current != NULL)
	{
		currentCollision = (Collision*)current->data;
		//Determine if this obj is obj1 or obj2
		if(obj == currentCollision->obj1)
		{
			//Make sure collision normal is pointing up
			if(currentCollision->minimumTranslationVector->components[1] == 1.0f)
			{
				onGround = 1;
				break;
			}
		}
		else
		{
			//Make sure collision normal is pointing down
			if(currentCollision->minimumTranslationVector->components[1] == -1.0f)
			{
				onGround = 1;
				break;
			}
		}

		current = current->next;
	}

	//If the object is allowed to jump
	if(onGround)
	{
		//Grab the state members
		struct State_RunnerController_Members* members = (struct State_RunnerController_Members*)state->members;


		Vector jumpImpulse;
		Vector_INIT_ON_STACK(jumpImpulse, 3);

		jumpImpulse.components[1] = members->jumpMag;

		RigidBody_ApplyImpulse(obj->body, &jumpImpulse, &Vector_ZERO);
	}
}