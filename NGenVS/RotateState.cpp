#include "RotateState.h"
#include "GObject.h"

#include "TimeManager.h"

struct State_Rotate_Members
{
	float angularVelocity;
	Vector* axis;
};

///
//Initializes a rotation state
//
//Parameters:
//	s: The state to be initialized as a rotation state
//	axis: The axis which this state will rotate it's object around
//	aVel: The angular velocity which this state will rotate it's object at
void State_Rotate_Initialize(State* s, const Vector* axis, const float aVel)
{
	s->members = (State_Members)malloc(sizeof(struct State_Rotate_Members));
	//Get members as a State_Rotate_Members struct
	struct State_Rotate_Members* members = (struct State_Rotate_Members*)s->members;

	members->axis = Vector_Allocate();
	Vector_Initialize(members->axis, 3);

	for (int i = 0; i < 3; i++)
		members->axis->components[i] = axis->components[i];

	members->angularVelocity = aVel;

	s->State_Update = State_Rotate_Update;
	s->State_Members_Free = State_Rotate_Free;
}

///
//Frees memory used by a Rotation State
//
//Parameters:
//	s: The rotation state to free
void State_Rotate_Free(State* s)
{
	//Get members as a State_Rotate_Members struct
	struct State_Rotate_Members* members = (struct State_Rotate_Members*)s->members;

	Vector_Free(members->axis);
	free(members);
}

///
//Rotates an object according to it's rotation state
//
//Parameters:
//	GO: The game object with a RotationState being rotated
//	state: The RotateState updating the GameObject
void State_Rotate_Update(GObject* GO, State* state)
{
	//Get members as a State_Rotate_Members struct
	struct State_Rotate_Members* members = (struct State_Rotate_Members*)state->members;

	long long dtl = TimeManager_GetTimeBuffer().deltaTime->QuadPart;
	float dt = (float)dtl / 1000000.0f;
	GObject_Rotate(GO, members->axis, members->angularVelocity * dt);
}
