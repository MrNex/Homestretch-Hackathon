#include "PhysicsManager.h"

#include <stdio.h>
#include <math.h>

#include "TimeManager.h"

///
//Allocates memory for a new Physics Buffer
//
//Returns:
//	Pointer to a newly allocated physics buffer
static PhysicsBuffer* PhysicsManager_AllocateBuffer()
{
	PhysicsBuffer* buffer = (PhysicsBuffer*)malloc(sizeof(PhysicsBuffer));
	return buffer;
}

///
//Initializes a physics buffer
//
//Parameters:
//	buffer: The physics buffer to initialize
static void PhysicsManager_InitializeBuffer(PhysicsBuffer* buffer)
{
	buffer->globalForces = LinkedList_Allocate();
	LinkedList_Initialize(buffer->globalForces);

	buffer->globalAccelerations = LinkedList_Allocate();
	LinkedList_Initialize(buffer->globalAccelerations);
}

///
//Frees memory taken by a physics buffer
//
//Parameters:
//	buffer: The buffer to free the memory of
static void PhysicsManager_FreeBuffer(PhysicsBuffer* buffer)
{
	//Delete all vectors in the list of global forces
	LinkedList_Node* current = buffer->globalForces->head;
	while(current != NULL)
	{
		Vector_Free((Vector*)current->data);
		current = current->next;
	}
	//Delete list of global forces
	LinkedList_Free(buffer->globalForces);

	//Delete all vectors in the list of global accelerations
	current = buffer->globalAccelerations->head;
	while(current != NULL)
	{
		Vector_Free((Vector*)current->data);
		current = current->next;
	}
	//Delete linked list of global accelerations
	LinkedList_Free(buffer->globalAccelerations);

	//Free the buffer itself
	free(buffer);
}

///
//Initializes the physics manager
void PhysicsManager_Initialize()
{
	//Allocate and initialize the buffer
	physicsBuffer = PhysicsManager_AllocateBuffer();
	PhysicsManager_InitializeBuffer(physicsBuffer);
}

///
//Frees internal data of the physics manager
void PhysicsManager_Free()
{
	PhysicsManager_FreeBuffer(physicsBuffer);
}

///
//Gets a reference to the internal physics buffer of the physics manager
//
//Returns:
//	A pointer to the physics buffer
PhysicsBuffer* PhysicsManager_GetPhysicsBuffer()
{
	return physicsBuffer;
}

///
//Updates the Rigidbody components of all gameObjects
//
//Parameters:
//	gameObjects: THe linked list of gameobjects to update their rigidBodies
void PhysicsManager_Update(LinkedList* gameObjects)
{
	PhysicsManager_UpdateBodies(gameObjects);
	PhysicsManager_UpdateObjects(gameObjects);
}

///
//Updates the Rigidbody components of all gameObjects
//
//Parameters:
//	gameObjects: THe linked list of gameobjects to update their rigidBodies
void PhysicsManager_UpdateBodies(LinkedList* gameObjects)
{
	struct LinkedList_Node* current = gameObjects->head;
	struct LinkedList_Node* next = NULL;
	GObject* gameObject = NULL;

	float dt = TimeManager_GetDeltaSec();
	while(current != NULL)
	{
		next = current->next;
		gameObject = (GObject*)current->data;
		if(gameObject->body != NULL)
		{
			if( gameObject->body->physicsOn)
			{
				PhysicsManager_ApplyGlobalForces(gameObject->body, dt);
				PhysicsManager_UpdateLinearPhysicsOfBody(gameObject->body, dt);
				PhysicsManager_UpdateRotationalPhysicsOfBody(gameObject->body, dt);
			}
		}


		current = next;

	}
}

///
//Applies all global forces to the given rigidbody
//
//Parameters:
//	body: The rigidbody to apply global forces to
//	dt: The change in type since last update
void PhysicsManager_ApplyGlobalForces(RigidBody* body, float dt)
{
	LinkedList_Node* currentNode = physicsBuffer->globalForces->head;
	Vector* currentForce;

	//Apply each global force
	while(currentNode != NULL)
	{
		currentForce = (Vector*)currentNode->data;

		RigidBody_ApplyForce(body, (const Vector*)currentForce, &Vector_ZERO);

		currentNode = currentNode->next;
	}

	//If the object does not have an infinite mass
	if(body->inverseMass != 0.0f)
	{
		Vector scaledForce;
		Vector_INIT_ON_STACK(scaledForce, 3);

		//Apply each global acceleration
		currentNode = physicsBuffer->globalAccelerations->head;
		while(currentNode != NULL)
		{
			currentForce = (Vector*)currentNode->data;
			Vector_GetScalarProduct(&scaledForce, currentForce, 1.0f / body->inverseMass);
			
			RigidBody_ApplyForce(body, &scaledForce, &Vector_ZERO);
			
			currentNode = currentNode->next;
		}
	}
}

///
//Updates the linear physics of a rigidbody
//This determines calculations of acceleration, velocity, and position from netForce and netImpulse
//
//Parameters:
//	body: The body to update
//	dt: The change in time since last update
void PhysicsManager_UpdateLinearPhysicsOfBody(RigidBody* body, float dt)
{
	//F = MA
	//A = 1/M * F
	Vector_GetScalarProduct(body->acceleration, body->netForce, body->inverseMass);
	//J = MV so V = 1/M * J
	//J = J * 1/M
	//Will apply J to V later
	Vector_Scale(body->netImpulse, body->inverseMass);	


	Vector VT;
	Vector_INIT_ON_STACK( VT , 3 );
	Vector AT;
	Vector_INIT_ON_STACK( AT , 3);
	Vector VAT2;
	Vector_INIT_ON_STACK( VAT2 , 3);

	//Get V0T
	Vector_GetScalarProduct(&VT, body->velocity, dt);			//VT = V0 * dt		

	//Get AT
	Vector_GetScalarProduct(&AT, body->acceleration, dt);		//AT = A1 * dt

	//Get AT^2
	Vector_GetScalarProduct(&VAT2, &AT, dt);	//VAT2 = A1 * dt ^ 2
	//Get 1/2AT^2
	Vector_Scale(&VAT2, 0.5f);	//VAT2 = 1/2 * A1 * dt ^ 2

	//Get VT + 1/2AT^2
	Vector_Increment(&VAT2, &VT);	//VAT2 = VT + 1/2 * A1 * dt^2

	//X = X0 + V0T + 1/2AT^2
	Vector_Increment(body->frame->position, &VAT2);
	//V = V0 + AT
	Vector_Increment(body->velocity, &AT);	
	//V += 1/M * J
	Vector_Increment(body->velocity, body->netImpulse);
}

///
//Updates the rotational physics of a rigidbody
//This determines calculations of angular acceleration, angular velocity, and orientation / rotation
//from net torque and inverse Inertia
//
//Parameters:
//	body: The body to update
//	dt: The change in time since last update
void PhysicsManager_UpdateRotationalPhysicsOfBody(RigidBody* body, float dt)
{
	Vector AT;
	Vector_INIT_ON_STACK(AT, 3);

	Vector VT;
	Vector_INIT_ON_STACK(VT, 3);


	//T = IA
	//1/I * T = A
	Matrix_GetProductVector(body->angularAcceleration, body->inverseInertia, body->netTorque);
	//Same for instantaneous torque, accept it is applied directly to angular velocity at the end of this function
	Matrix_TransformVector(body->inverseInertia, body->netInstantaneousTorque);

	//A = dV / dT
	//A * dT = dV
	Vector_GetScalarProduct(&AT, body->angularAcceleration, dt);

	//A * dT = (V1 - V0)
	//A * dT + V0 = V1
	Vector_Increment(body->angularVelocity, &AT);
	//Apply instantaneousTorque to angularVelocity
	Vector_Increment(body->angularVelocity, body->netInstantaneousTorque);

	//V = dTheta / dt
	//V * dT = dTheta
	//V * dT = (Theta1 - Theta0)
	//V * dT + Theta0 = Theta1
	Vector_GetScalarProduct(&VT, body->angularVelocity, dt);

	//Rotate by |VT| around axis VT
	float theta = Vector_GetMag(&VT);
	Vector_Normalize(&VT);

	if(theta != 0)
	{
		FrameOfReference_Rotate(body->frame, &VT, theta);
	}

}

///
//Updates the Frame of reference component of all gameObjects to match their rigidbodies
//
//Parameters:
//	gameObjects: the linked list of gameObjects to update their rigidbodies
void PhysicsManager_UpdateObjects(LinkedList* gameObjects)
{
	float dt = TimeManager_GetDeltaSec();
	struct LinkedList_Node* current = gameObjects->head;
	struct LinkedList_Node* next = NULL;
	GObject* gameObject = NULL;


	while(current != NULL)
	{
		next = current->next;
		gameObject = (GObject*)current->data;
		if(gameObject->body != NULL)
		{
			if( gameObject->body->physicsOn)
			{
				Vector_Copy(gameObject->frameOfReference->position, gameObject->body->frame->position);
				Matrix_Copy(gameObject->frameOfReference->rotation, gameObject->body->frame->rotation);

				//Update previous net force
				Vector_GetScalarProduct(gameObject->body->previousNetForce, gameObject->body->netForce, dt);
				Vector_Increment(gameObject->body->previousNetForce, gameObject->body->netImpulse);

				//Update previous net torque
				Vector_GetScalarProduct(gameObject->body->previousNetTorque, gameObject->body->netTorque, dt);
				Vector_Increment(gameObject->body->previousNetTorque, gameObject->body->netInstantaneousTorque);

				//Set netforce back to 0
				Vector_Copy(gameObject->body->netForce, &Vector_ZERO);
				Vector_Copy(gameObject->body->acceleration, &Vector_ZERO);
				Vector_Copy(gameObject->body->netImpulse, &Vector_ZERO);
				Vector_Copy(gameObject->body->netTorque, &Vector_ZERO);
				Vector_Copy(gameObject->body->netInstantaneousTorque, &Vector_ZERO);
			}
		}


		current = next;

	}
}


///
//Resolves all collisions in a linked list
//
//Parameters:
//	collisions: A linked list of all collisions detected which need resolving
void PhysicsManager_ResolveCollisions(LinkedList* collisions)
{
	//Loop through the linked list of collisions
	LinkedList_Node* current = collisions->head;
	LinkedList_Node* next = NULL;

	Collision* collision;
	while(current != NULL)
	{
		next = current->next;
		collision = (Collision*)current->data;
		PhysicsManager_ResolveCollision(collision);



		current = next;


	}
}

///
//Resolves a collision
//
//Parameters:
//	collisions: The collision which needs to be resolved
static void PhysicsManager_ResolveCollision(Collision* collision)
{
	//Check if the collision must be resolved
	if(PhysicsManager_IsResolutionNeeded(collision))
	{
		//printf("MTV:\t");
		//Vector_PrintTranspose(collision->minimumTranslationVector);
		//printf("Resolving\n");

		//Step 1: Use the minimum translation vector to pull the intersecting objects apart
		//If and only if the objects need to be pulled apart
		PhysicsManager_DecoupleCollision(collision);

		//Step 2: Calculate the point of collision
		//In the case that one of the objects is an AABB, there will be different collision points. Allocate an array of two vectors to store these points.
		Vector* pointsOfCollision[2];

		//Allocate and initialize these vectors individually
		pointsOfCollision[0] = Vector_Allocate();
		pointsOfCollision[1] = Vector_Allocate();

		Vector_Initialize(pointsOfCollision[0], 3);
		Vector_Initialize(pointsOfCollision[1], 3);

		PhysicsManager_DetermineCollisionPoints(pointsOfCollision, collision);


		//Step 3: Calculate and apply impulses due to collision
		PhysicsManager_ApplyCollisionImpulses(collision, (const Vector**)pointsOfCollision);


		//Step 4a: Calculate frictional coefficients
		//float staticCoefficient = sqrt(powf(collision->obj1->body != NULL ? collision->obj1->body->staticFriction : 1.0f, 2)+powf(collision->obj2->body != NULL ? collision->obj2->body->staticFriction : 1.0f, 2));
		//float dynamicCoefficient = sqrt(powf(collision->obj1->body != NULL ? collision->obj1->body->dynamicFriction : 1.0f, 2)+powf(collision->obj2->body != NULL ? collision->obj2->body->dynamicFriction : 1.0f, 2));

		float staticCoefficient = ((collision->obj1->body != NULL ? collision->obj1->body->staticFriction : 1.0f) + (collision->obj2->body != NULL ? collision->obj2->body->staticFriction : 1.0f)) / 2.0f;
		float dynamicCoefficient = ((collision->obj1->body != NULL ? collision->obj1->body->dynamicFriction : 1.0f) + (collision->obj2->body != NULL ? collision->obj2->body->dynamicFriction : 1.0f)) / 2.0f;

		//printf("static:\t%f\ndynamic:\t%f\n", staticCoefficient, dynamicCoefficient);

		//Step 4b: Calculate and apply frictional impulses
		PhysicsManager_ApplyLinearFrictionalImpulses(collision, (const Vector**)pointsOfCollision, staticCoefficient, dynamicCoefficient);
		PhysicsManager_ApplyFrictionalTorques(collision, staticCoefficient, dynamicCoefficient);

		//Step 5:
		/*
		float dt = TimeManager_GetDeltaSec();

		//And update the objects in this collision
		if(collision->obj1->body != NULL)
		{
		PhysicsManager_ApplyGlobalForces(collision->obj1->body, dt);

		PhysicsManager_UpdateLinearPhysicsOfBody(collision->obj1->body, dt);
		PhysicsManager_UpdateRotationalPhysicsOfBody(collision->obj1->body, dt);
		}

		if(collision->obj2->body != NULL)
		{
		PhysicsManager_ApplyGlobalForces(collision->obj2->body, dt);

		PhysicsManager_UpdateLinearPhysicsOfBody(collision->obj2->body, dt);
		PhysicsManager_UpdateRotationalPhysicsOfBody(collision->obj2->body, dt);
		}
		*/

		//Free the vectors used to hold the collision points
		Vector_Free(pointsOfCollision[0]);
		Vector_Free(pointsOfCollision[1]);
	}


}

///
//TODO: Account for linear velocity at closest point due to angular velocity.
//Determines if a collision needs to be resolved, or if it is resolving itself
//
//PArameters:
//	collision: The collision to test
//
//REturns:
//	0 if no collision resolution is needed
//	1 if collision resolution is needed
static unsigned char PhysicsManager_IsResolutionNeeded(Collision* collision)
{
	//If the overlap is 0 (contact case) or negative, this collision does not need resolving
	if(collision->overlap <= 0.0f)
	{
		return 0;	
	}

	//If both of the objects have velocities
	if(collision->obj1->body && collision->obj2->body)
	{
		//Calculate the total linear velocities of the points on objects A and B most in the direction of their relative MTV's
		Vector totalVelocity1;
		Vector totalVelocity2;

		Vector_INIT_ON_STACK(totalVelocity1, 3);
		Vector_INIT_ON_STACK(totalVelocity2, 3);

		//Calculate Obj1's total velocity of point on obj1 furthest in direction of relative MTV
		if(collision->obj1->collider->type == COLLIDER_CONVEXHULL)
		{
			Vector relativeMTV;
			Vector_INIT_ON_STACK(relativeMTV, 3);
			Vector_GetScalarProduct(&relativeMTV, collision->minimumTranslationVector, -1.0f);
			//Determine the set of points most in the direction of the relative MTV

			//Grab convexHull data set
			ColliderData_ConvexHull* convex = collision->obj1->collider->data->convexHullData;

			//Create an array of pointers to vectors to hold the model oriented collider points
			Vector** modelOrientedPoints = (Vector**)malloc(sizeof(Vector*) * convex->points->size);
			//Allocate & initialize individual vectors in array
			for(unsigned int i = 0; i < convex->points->size; i++)
			{
				modelOrientedPoints[i] = Vector_Allocate();
				Vector_Initialize(modelOrientedPoints[i], 3);
			}

			//Get the model oriented points
			ConvexHullCollider_GetOrientedModelPoints(modelOrientedPoints, convex, collision->obj1Frame);

			//Allocate & initialize a dynamic array to hold the points furthest in the direction of the MTV
			DynamicArray* furthestPoints = DynamicArray_Allocate();
			DynamicArray_Initialize(furthestPoints, sizeof(Vector));

			//Get the furthest points in the direction of relative MTV
			ConvexHullCollider_GetFurthestPoints(furthestPoints, convex, (const Vector**) modelOrientedPoints, &relativeMTV);

			//Calculate the maximum linear velocity due to the bodies angular velocity in the direction of the relative MTV
			Vector maxLinearVelocityAtPoint;
			Vector_INIT_ON_STACK(maxLinearVelocityAtPoint, 3);

			RigidBody_CalculateMaxLocalLinearVelocity(&maxLinearVelocityAtPoint, collision->obj1->body, furthestPoints, &relativeMTV);

			//Once the maximum linear velocity due to angular velocity of the furthest points is found, inrement the total velocity by that
			Vector_Increment(&totalVelocity1, &maxLinearVelocityAtPoint);



			//Now free the data used for this algorithm
			DynamicArray_Free(furthestPoints);

			for(unsigned int i = 0; i < convex->points->size; i++)
			{
				Vector_Free(modelOrientedPoints[i]);
			}
			free(modelOrientedPoints);
		}

		//Increment the total velocity by the linear velocity of the object
		Vector_Increment(&totalVelocity1, collision->obj1->body->velocity);


		//Calculate Obj2's total velocity of point on obj2 furthest in direction of relative MTV
		if(collision->obj2->collider->type == COLLIDER_CONVEXHULL)
		{
			//Determine the set of points most in the direction of the relative MTV

			//Grab convexHull data set
			ColliderData_ConvexHull* convex = collision->obj2->collider->data->convexHullData;

			//Create an array of pointers to vectors to hold the model oriented collider points
			Vector** modelOrientedPoints = (Vector**)malloc(sizeof(Vector*) * convex->points->size);
			//Allocate & initialize individual vectors in array
			for(unsigned int i = 0; i < convex->points->size; i++)
			{
				modelOrientedPoints[i] = Vector_Allocate();
				Vector_Initialize(modelOrientedPoints[i], 3);
			}

			//Get the model oriented points
			ConvexHullCollider_GetOrientedModelPoints(modelOrientedPoints, convex, collision->obj2Frame);

			//Allocate & initialize a dynamic array to hold the points furthest in the direction of the MTV
			DynamicArray* furthestPoints = DynamicArray_Allocate();
			DynamicArray_Initialize(furthestPoints, sizeof(Vector));

			//Get the furthest points in the direction of relative MTV
			ConvexHullCollider_GetFurthestPoints(furthestPoints, convex, (const Vector**) modelOrientedPoints, collision->minimumTranslationVector);

			Vector maxLinearVelocityAtPoint;
			Vector_INIT_ON_STACK(maxLinearVelocityAtPoint, 3);

			//Calculate the maximum linear velocity due to the body's angular velocity in the direction of the relative MTV
			RigidBody_CalculateMaxLocalLinearVelocity(&maxLinearVelocityAtPoint, collision->obj2->body, furthestPoints, collision->minimumTranslationVector);

			//Once the maximum linear velocity due to angular velocity of the furthest points is found, inrement the total velocity by that
			Vector_Increment(&totalVelocity2, &maxLinearVelocityAtPoint);


			//Now free the data used for this algorithm
			DynamicArray_Free(furthestPoints);

			for(unsigned int i = 0; i < convex->points->size; i++)
			{
				Vector_Free(modelOrientedPoints[i]);
			}
			free(modelOrientedPoints);
		}


		//Increment the total velocity by the linear velocity of the object
		Vector_Increment(&totalVelocity2, collision->obj2->body->velocity);


		//Find the relative velocity of object B from a point on object A
		Vector relVelocity;
		Vector_INIT_ON_STACK(relVelocity, 3);

		//Vector_Subtract(&relVelocity, collision->obj2->body->velocity, collision->obj1->body->velocity);
		Vector_Subtract(&relVelocity, &totalVelocity2, &totalVelocity1);

		Vector_Normalize(&relVelocity);
		Vector_Normalize(collision->minimumTranslationVector);

		//If the relative velocity is in the direction of the MTV object, object B is smashing into object A
		if(Vector_DotProduct(&relVelocity, collision->minimumTranslationVector) > 0.0f)
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}

	//Obj1's velocity should be in the directon of the MTV if it is moving away from obj2
	if(collision->obj1->body != NULL)
	{
		Vector totalVelocity1;
		Vector_INIT_ON_STACK(totalVelocity1, 3);

		//Calculate Obj1's total velocity of point on obj1 furthest in direction of relative MTV
		if(collision->obj1->collider->type == COLLIDER_CONVEXHULL)
		{
			//Determine the point furthest in the direction of the relative MTV
			Vector relativeMTV;
			Vector_INIT_ON_STACK(relativeMTV, 3);
			Vector_GetScalarProduct(&relativeMTV, collision->minimumTranslationVector, -1.0f);

			//Determine the set of points most in the direction of the relative MTV

			//Grab convexHull data set
			ColliderData_ConvexHull* convex = collision->obj1->collider->data->convexHullData;

			//Create an array of pointers to vectors to hold the model oriented collider points
			Vector** modelOrientedPoints = (Vector**)malloc(sizeof(Vector*) * convex->points->size);
			//Allocate & initialize individual vectors in array
			for(unsigned int i = 0; i < convex->points->size; i++)
			{
				modelOrientedPoints[i] = Vector_Allocate();
				Vector_Initialize(modelOrientedPoints[i], 3);
			}

			//Get the model oriented points
			ConvexHullCollider_GetOrientedModelPoints(modelOrientedPoints, convex, collision->obj1Frame);

			//Allocate & initialize a dynamic array to hold the points furthest in the direction of the MTV
			DynamicArray* furthestPoints = DynamicArray_Allocate();
			DynamicArray_Initialize(furthestPoints, sizeof(Vector));

			//Get the furthest points in the direction of relative MTV
			ConvexHullCollider_GetFurthestPoints(furthestPoints, convex, (const Vector**) modelOrientedPoints, &relativeMTV);

			Vector maxLinearVelocityAtPoint;
			Vector_INIT_ON_STACK(maxLinearVelocityAtPoint, 3);

			//Calculate the maximum linear velocity due to the body's angular velocity in the direction of the relative MTV
			RigidBody_CalculateMaxLocalLinearVelocity(&maxLinearVelocityAtPoint, collision->obj1->body, furthestPoints, &relativeMTV);

			//Once the maximum linear velocity due to angular velocity of the furthest points is found, inrement the total velocity by that
			Vector_Increment(&totalVelocity1, &maxLinearVelocityAtPoint);

			//Now free the data used for this algorithm
			DynamicArray_Free(furthestPoints);

			for(unsigned int i = 0; i < convex->points->size; i++)
			{
				Vector_Free(modelOrientedPoints[i]);
			}
			free(modelOrientedPoints);
		}

		//Increment the total velocity by the linear velocity of object1
		Vector_Increment(&totalVelocity1, collision->obj1->body->velocity);

		if(Vector_DotProduct(&totalVelocity1, collision->minimumTranslationVector) < 0.0f)
		{
			return 1;
		}
	}
	//Obj2's velocity should be opposite the direction of the MTV if it is moving away from obj1
	if(collision->obj2->body != NULL)
	{
		Vector totalVelocity2;
		Vector_INIT_ON_STACK(totalVelocity2, 3);

		//Calculate Obj2's total velocity of point on obj2 furthest in direction of relative MTV
		if(collision->obj2->collider->type == COLLIDER_CONVEXHULL)
		{

			//Determine the set of points most in the direction of the relative MTV

			//Grab convexHull data set
			ColliderData_ConvexHull* convex = collision->obj2->collider->data->convexHullData;

			//Create an array of pointers to vectors to hold the model oriented collider points
			Vector** modelOrientedPoints = (Vector**)malloc(sizeof(Vector*) * convex->points->size);
			//Allocate & initialize individual vectors in array
			for(unsigned int i = 0; i < convex->points->size; i++)
			{
				modelOrientedPoints[i] = Vector_Allocate();
				Vector_Initialize(modelOrientedPoints[i], 3);
			}

			//Get the model oriented points
			ConvexHullCollider_GetOrientedModelPoints(modelOrientedPoints, convex, collision->obj2Frame);

			//Allocate & initialize a dynamic array to hold the points furthest in the direction of the MTV
			DynamicArray* furthestPoints = DynamicArray_Allocate();
			DynamicArray_Initialize(furthestPoints, sizeof(Vector));

			//Get the furthest points in the direction of relative MTV
			ConvexHullCollider_GetFurthestPoints(furthestPoints, convex, (const Vector**) modelOrientedPoints, collision->minimumTranslationVector);

			Vector maxLinearVelocityAtPoint;
			Vector_INIT_ON_STACK(maxLinearVelocityAtPoint, 3);

			//Calculate the maximum linear velocity due to the body's angular velocity in the direction of the relative MTV
			RigidBody_CalculateMaxLocalLinearVelocity(&maxLinearVelocityAtPoint, collision->obj2->body, furthestPoints, collision->minimumTranslationVector);

			//Once the maximum linear velocity due to angular velocity of the furthest points is found, inrement the total velocity by that
			Vector_Increment(&totalVelocity2, &maxLinearVelocityAtPoint);

			//Now free the data used for this algorithm
			DynamicArray_Free(furthestPoints);

			for(unsigned int i = 0; i < convex->points->size; i++)
			{
				Vector_Free(modelOrientedPoints[i]);
			}
			free(modelOrientedPoints);
		}


		//Increment total velocity of object 2 by linear velocity of object 2
		Vector_Increment(&totalVelocity2, collision->obj2->body->velocity);

		if(Vector_DotProduct(&totalVelocity2, collision->minimumTranslationVector) > 0.0f)
		{
			return 1;
		}
	}

	//If both objects are moving away from each other, no collision resolution is needed
	return 0;
}

///
//Takes two intersecting objects and restores them to the moment their surfaces touched
//
//Parameters:
//	collision: The collision to decouple
static void PhysicsManager_DecoupleCollision(Collision* collision)
{
	Vector resolutionVector1;	//The tranlation vector to get object 1 out of object 2
	Vector resolutionVector2;	//The translation vector to get object 2 out of object 1

	Vector_INIT_ON_STACK(resolutionVector1, 3);
	Vector_INIT_ON_STACK(resolutionVector2, 3);

	//If object 1 involved in collision has rigidbody physics enabled
	if(collision->obj1->body != NULL)
	{
		Vector_Copy(&resolutionVector1, collision->obj1->body->velocity);

		//Below here is experimental
		//TODO: Finish experiment

		//If the object's collider can be spinning
		if(collision->obj1->collider->type == COLLIDER_CONVEXHULL)
		{
			//If the collider IS spinning
			if(Vector_GetMag(collision->obj1->body->angularVelocity) > 0.0f)
			{
				//Determine the point furthest in the direction of the relative MTV
				Vector relativeMTV;
				Vector_INIT_ON_STACK(relativeMTV, 3);
				Vector_GetScalarProduct(&relativeMTV, collision->minimumTranslationVector, -1.0f);


				//Determine the set of points most in the direction of the relative MTV

				//Grab convexHull data set
				ColliderData_ConvexHull* convex = collision->obj1->collider->data->convexHullData;

				//Create an array of pointers to vectors to hold the model oriented collider points
				Vector** modelOrientedPoints = (Vector**)malloc(sizeof(Vector*) * convex->points->size);
				//Allocate & initialize individual vectors in array
				for(unsigned int i = 0; i < convex->points->size; i++)
				{
					modelOrientedPoints[i] = Vector_Allocate();
					Vector_Initialize(modelOrientedPoints[i], 3);
				}

				//Get the model oriented points
				ConvexHullCollider_GetOrientedModelPoints(modelOrientedPoints, convex, collision->obj1Frame);

				//Allocate & initialize a dynamic array to hold the points furthest in the direction of the MTV
				DynamicArray* furthestPoints = DynamicArray_Allocate();
				DynamicArray_Initialize(furthestPoints, sizeof(Vector));

				//Get the furthest points in the direction of relative MTV
				ConvexHullCollider_GetFurthestPoints(furthestPoints, convex, (const Vector**) modelOrientedPoints, &relativeMTV);

				Vector maxLinearVelocityAtPoint;
				Vector_INIT_ON_STACK(maxLinearVelocityAtPoint, 3);

				//Calculate the maximum linear velocity due to the body's angular velocity in the direction of the relative MTV
				RigidBody_CalculateMaxLocalLinearVelocity(&maxLinearVelocityAtPoint, collision->obj1->body, furthestPoints, &relativeMTV);

				//Once the maximum linear velocity due to angular velocity of the furthest points is found, inrement the resolution vector by that
				Vector_Increment(&resolutionVector1, &maxLinearVelocityAtPoint);

				//Now free the data used for this algorithm
				DynamicArray_Free(furthestPoints);

				for(unsigned int i = 0; i < convex->points->size; i++)
				{
					Vector_Free(modelOrientedPoints[i]);
				}
				free(modelOrientedPoints);
			}

		}
	}
	else
	{
		Vector_Copy(&resolutionVector1, &Vector_ZERO);
	}

	//If object 2 involved in collision has rigidbody physics enabled
	if(collision->obj2->body != NULL)
	{
		Vector_Copy(&resolutionVector2, collision->obj2->body->velocity);

		//If the object's collider can be spinning
		if(collision->obj2->collider->type == COLLIDER_CONVEXHULL)
		{

			//Determine the set of points most in the direction of the relative MTV

			//Grab convexHull data set
			ColliderData_ConvexHull* convex = collision->obj2->collider->data->convexHullData;

			//Create an array of pointers to vectors to hold the model oriented collider points
			Vector** modelOrientedPoints = (Vector**)malloc(sizeof(Vector*) * convex->points->size);
			//Allocate & initialize individual vectors in array
			for(unsigned int i = 0; i < convex->points->size; i++)
			{
				modelOrientedPoints[i] = Vector_Allocate();
				Vector_Initialize(modelOrientedPoints[i], 3);
			}

			//Get the model oriented points
			ConvexHullCollider_GetOrientedModelPoints(modelOrientedPoints, convex, collision->obj2Frame);

			//Allocate & initialize a dynamic array to hold the points furthest in the direction of the MTV
			DynamicArray* furthestPoints = DynamicArray_Allocate();
			DynamicArray_Initialize(furthestPoints, sizeof(Vector));

			//Get the furthest points in the direction of relative MTV
			ConvexHullCollider_GetFurthestPoints(furthestPoints, convex, (const Vector**) modelOrientedPoints, collision->minimumTranslationVector);

			Vector maxLinearVelocityAtPoint;
			Vector_INIT_ON_STACK(maxLinearVelocityAtPoint, 3);

			//Calculate the maximum linear velocity due to the body's angular velocity in the direction of the relative MTV
			RigidBody_CalculateMaxLocalLinearVelocity(&maxLinearVelocityAtPoint, collision->obj2->body, furthestPoints, collision->minimumTranslationVector);

			//Once the maximum linear velocity due to angular velocity of the furthest points is found, inrement the resolution vector by that
			Vector_Increment(&resolutionVector2, &maxLinearVelocityAtPoint);

			//Now free the data used for this algorithm
			DynamicArray_Free(furthestPoints);

			for(unsigned int i = 0; i < convex->points->size; i++)
			{
				Vector_Free(modelOrientedPoints[i]);
			}
			free(modelOrientedPoints);

		}
	}
	else
	{
		Vector_Copy(&resolutionVector2, &Vector_ZERO);
	}

	Vector_Scale(&resolutionVector1, -1.0f);
	Vector_Scale(&resolutionVector2, -1.0f);

	//Project the resolution vectors (containing - velocity) onto the MTV
	Vector_Project(&resolutionVector1, collision->minimumTranslationVector);
	Vector_Project(&resolutionVector2, collision->minimumTranslationVector);

	//Find the scale values for the resolution vectors
	//These scales should be the mass of the object / sum of the masses
	float scale1, scale2;

	//If obj1 has no rigid body, it cannot have velocity and therefore object 2 must do
	//All work to decouple the collision
	if(collision->obj1->body == NULL)
	{
		scale1 = 0.0f;
		scale2 = 1.0f;
	}
	else if(collision->obj2->body == NULL)
	{
		scale1 = 1.0f;
		scale2 = 0.0f;
	}
	//If an object has infinite mass, or no velocity, it cannot move and therefore the other object
	//Must do all work to decouple the collision
	else if(collision->obj1->body->inverseMass == 0.0f || Vector_GetMag(&resolutionVector1) == 0.0f)
	{
		scale1 = 0.0f;
		scale2 = 1.0f;
	}
	else if(collision->obj2->body->inverseMass == 0.0f || Vector_GetMag(&resolutionVector2) == 0.0f)
	{
		scale1 = 1.0f;
		scale2 = 0.0f;
	}
	//Both objects can and are moving, so they can both do a portion of work to decouple the collision
	else
	{
		//Calculate the amount each object should move to decouple the collision
		//As a function of the ratio of the objects mass to the total mass 
		//& the speeds the objects are moving (Which will be applied later)
		scale1 = (collision->obj1->body->inverseMass * collision->obj2->body->inverseMass) / (collision->obj1->body->inverseMass + collision->obj2->body->inverseMass);
		scale2 = scale1;
		scale1 /= collision->obj2->body->inverseMass;
		scale2 /= collision->obj1->body->inverseMass;

	}


	if(collision->overlap > 0.0f)
	{
		//Scale the scales by the magnitude of the MTV overlap
		scale1 *= collision->overlap;
		scale2 *= collision->overlap;

		//Normalize the resolution vectors
		Vector_Normalize(&resolutionVector1);
		Vector_Normalize(&resolutionVector2);

		//Scale the resolution vectors by respective scale values
		Vector_Scale(&resolutionVector1, scale1);
		Vector_Scale(&resolutionVector2, scale2);

		//In the case the collision is being resolved by the objects moving in the same direction...
		if(Vector_DotProduct(&resolutionVector1, &resolutionVector2) > 0.0f)
		{
			Vector displacement;
			Vector_INIT_ON_STACK(displacement, 3);
			//Get vector pointing from b to a
			Vector_Subtract(&displacement, collision->obj1Frame->position, collision->obj2Frame->position);

			//Make sure resolution1 points toward it
			if(Vector_DotProduct(&resolutionVector1, &displacement) < 0)
			{
				Vector_Scale(&resolutionVector1, -1.0f);
			}
			else if(Vector_DotProduct(&resolutionVector2, &displacement) > 0)
			{
				Vector_Scale(&resolutionVector2, -1.0f);
			}

		}

		//Translate each object's frame of reference by their resolution vectors (if they have a rigidbody)
		if(collision->obj1->body != NULL)
		{
			FrameOfReference_Translate(collision->obj1->body->frame, &resolutionVector1);

		}
		if(collision->obj2->body != NULL)
		{
			FrameOfReference_Translate(collision->obj2->body->frame, &resolutionVector2);

		}
	}
}

///
//Determines the point of contact in a decoupled collision
//In the case where at least one of the objects is an AABB, there will be two separate collision points,
//One on each object.
//
//Parameters:
//	dest: Array of two Vectors to store the points of collision in. These are the collision points of 
//		obj1 and obj2 respectively.
//	collision: The collision to determine the point of contact 
static void PhysicsManager_DetermineCollisionPoints(Vector** dest, Collision* collision)
{
	unsigned char obj1PointFound = 0;	//Boolean dictating if obj1's collision point was determined
	unsigned char obj2PointFound = 0;	//Boolean dictating if obj2's collision point was determined

	//If either object is a sphere, first do the sphere test as it's the least expensive & most accurate
	if(collision->obj2->collider->type == COLLIDER_SPHERE)
	{
		PhysicsManager_DetermineCollisionPointSphere(dest[1], collision->obj2->collider->data->sphereData, collision->obj2Frame, collision->minimumTranslationVector);
		obj2PointFound = 1;

		//If obj2 is a sphere and obj1 is anything but an AABB, they have the same collision point
		if(collision->obj1->collider->type != COLLIDER_AABB)
		{
			Vector_Copy(dest[0], dest[1]);
			obj1PointFound = 1;
		}
	}
	else if(collision->obj1->collider->type == COLLIDER_SPHERE)
	{
		//Obj1's relative MTV is the negated MTV, because the MTV always points towards obj1 by convention
		Vector relativeMTV;
		Vector_INIT_ON_STACK(relativeMTV, 3);
		Vector_GetScalarProduct(&relativeMTV, collision->minimumTranslationVector, -1.0f);

		PhysicsManager_DetermineCollisionPointSphere(dest[0], collision->obj1->collider->data->sphereData, collision->obj1Frame, &relativeMTV);
		obj1PointFound = 1;

		//If obj1 is a sphere and obj2 is anything but an AABB, they have the same collision point
		if(collision->obj2->collider->type != COLLIDER_AABB)
		{
			Vector_Copy(dest[1], dest[0]);
			obj2PointFound = 1;
		}
	}

	//Exit here if both points have been found
	if(obj1PointFound && obj2PointFound)
	{
		return;
	}

	//Check if obj1 is an AABB
	if(collision->obj1->collider->type == COLLIDER_AABB)
	{
		PhysicsManager_DetermineCollisionPointAABB(dest[0], collision->obj1Frame);
		obj1PointFound = 1;
	}

	//Check if obj2 is an AABB
	if(collision->obj2->collider->type == COLLIDER_AABB)
	{
		PhysicsManager_DetermineCollisionPointAABB(dest[1], collision->obj2Frame);
		obj2PointFound = 1;
	}

	//Exit here if both points have been found
	if(obj1PointFound && obj2PointFound)
	{
		return;
	}

	//Only cases which involve 1 or more convex hulls & no spheres have not been determined yet.
	//If we have 1 of the points we have an AABB - Convex Hull case
	if(obj1PointFound || obj2PointFound)
	{
		//Determine which object has yet to have it's point found (This is the convex hull object)
		if(!obj1PointFound)
		{
			//If obj1 is convex we must convert obj2 to be convex as well.
			ColliderData_ConvexHull* AABBAsConvex = ConvexHullCollider_AllocateData();
			ConvexHullCollider_InitializeData(AABBAsConvex);

			AABBCollider_ToConvexHullCollider(AABBAsConvex, collision->obj2->collider->data->AABBData);

			//Next we must get obj1's relative MTV
			Vector relativeMTV;
			Vector_INIT_ON_STACK(relativeMTV, 3);
			Vector_GetScalarProduct(&relativeMTV, collision->minimumTranslationVector, -1.0f);

			//Now we can determine the collision point of obj1 using the convex hull method
			PhysicsManager_DetermineCollisionPointConvexHull(dest[0], 
				collision->obj1->collider->data->convexHullData, collision->obj1Frame, AABBAsConvex, collision->obj2Frame, 
				&relativeMTV);

			obj1PointFound = 1;

			//Free data allocated for representing obj2 as a convex hull
			ConvexHullCollider_FreeData(AABBAsConvex);
		}
		else
		{
			//If obj2 is convex we must convert obj1 to be convex as well
			ColliderData_ConvexHull* AABBAsConvex = ConvexHullCollider_AllocateData();
			ConvexHullCollider_InitializeData(AABBAsConvex);

			AABBCollider_ToConvexHullCollider(AABBAsConvex, collision->obj1->collider->data->AABBData);

			//Now we can determine the collision point of obj2 using the convex hull method
			PhysicsManager_DetermineCollisionPointConvexHull(dest[1], 
				collision->obj2->collider->data->convexHullData, collision->obj2Frame, AABBAsConvex, collision->obj1Frame, 
				collision->minimumTranslationVector);

			obj2PointFound = 1;

			//Free data allocated for representing obj1 as a convex hull
			ConvexHullCollider_FreeData(AABBAsConvex);
		}
	}
	//Else there is a convex hull - convex hull case
	else
	{
		//Grab the convex hull data from both colliders
		ColliderData_ConvexHull* convex1 = collision->obj1->collider->data->convexHullData;
		ColliderData_ConvexHull* convex2 = collision->obj2->collider->data->convexHullData;

		//Determine obj2's collision point
		PhysicsManager_DetermineCollisionPointConvexHull(dest[1], convex2, collision->obj2Frame, convex1, collision->obj1Frame, collision->minimumTranslationVector);
		obj2PointFound = 1;

		//Obj1's collision point will be the same
		Vector_Copy(dest[0], dest[1]);
		obj1PointFound = 1;
	}

}

///
//Determines the point of contact on a sphere collider
//
//Parameters:
//	dest: A pointer to a vector to store the collision point in.
//	sphere: A pointer to the sphere collider data involved in collision
//	sphereFrame: A pointer to the frame of reference of the sphere involved in the collision
//	relativeMTV: A pointer to a vector representing the MTV !pointing towards the other object!
static void PhysicsManager_DetermineCollisionPointSphere(Vector* dest, const ColliderData_Sphere* sphere, const FrameOfReference* sphereFrame, const Vector* relativeMTV)
{
	//GEt the scaled radius of the sphere collider
	float scaledRadius = SphereCollider_GetScaledRadius(sphere, sphereFrame);

	//Scale relative MTV by radius to find collision point in model space
	Vector_GetScalarProduct(dest, relativeMTV, scaledRadius);

	//Translate collision point into worldspace to get final collision point.
	Vector_Increment(dest, sphereFrame->position);
}

///
//Determines the point of contact on an AABB Collider
//For an AABB the point of contact will always be the center of mass to prevent rotation of the object
//
//Parameters:
//	dest: A pointer to a vector to store the collision point in
//	AABBFrame: The frame of reference of the AABB involved in the collision
static void PhysicsManager_DetermineCollisionPointAABB(Vector* dest, const FrameOfReference* AABBFrame)
{
	//The collisiion point on an object which must prevent from rotating must be the center of mass in worldspace
	Vector_Copy(dest, AABBFrame->position);
}

///
//Determines the point of contact on a Convex Hull Collider
//
//PArameters:
//	dest: A pointer to a vector to store the collision point in
//	convexHull1: A pointer to the convex hull collider data involved in the collision which we are determining the point of collision on.
//	convexFrame1: A pointer to the frame of reference of the convex hull involved in the collision which we are determining the point of collision on.
//	convexHull2: A pointer to the other convex hull collider data involved in the collision, whether the object itself has a convex hull or not.
//	convexFrame2: A pointer to the frame of reference of the other convex hull involved in the collision, whether the object itself has a convex hull or not
//	relativeMTV: A pointer to a vector representing the MTV !Pointing towards the OTHER object (convexHull2)!
static void PhysicsManager_DetermineCollisionPointConvexHull(Vector* dest,
															 const ColliderData_ConvexHull* convexHull1, const FrameOfReference* convexFrame1, 
															 const ColliderData_ConvexHull* convexHull2, const FrameOfReference* convexFrame2,
															 const Vector* relativeMTV)
{
	//Create an unsigned character to serve as a boolean for whether the collision point was found yet
	unsigned char found = 0;

	//allocate arrays of vectors to hold the model space oriented points of colliders
	Vector** modelOrientedPoints1 = (Vector**)malloc(sizeof(Vector*) * convexHull1->points->size);
	Vector** modelOrientedPoints2 = (Vector**)malloc(sizeof(Vector*) * convexHull2->points->size);

	//Allocate & initialize individual vectors in array
	for(unsigned int i = 0; i < convexHull1->points->size; i++)
	{
		modelOrientedPoints1[i] = Vector_Allocate();
		Vector_Initialize(modelOrientedPoints1[i], 3);
	}

	//Allocate & initialize individual vectors in array
	for(unsigned int i = 0; i < convexHull2->points->size; i++)
	{
		modelOrientedPoints2[i] = Vector_Allocate();
		Vector_Initialize(modelOrientedPoints2[i], 3);
	}


	//Get the points of the collider oriented in modelSpace
	ConvexHullCollider_GetOrientedModelPoints(modelOrientedPoints1, convexHull1, convexFrame1);
	ConvexHullCollider_GetOrientedModelPoints(modelOrientedPoints2, convexHull2, convexFrame2);


	//We must now determine the type of collision:
	//Point - Point / Edge / Face
	//Edge - Edge
	//Edge - Face
	//OR Face - Face
	//
	//We can do this by finding the number of modelOriented points furthest in the direction of the respective relative MTV for each object
	//Allocate & Initialize Dynamic arrays to hold these points
	DynamicArray* furthestPoints1 = DynamicArray_Allocate();
	DynamicArray* furthestPoints2 = DynamicArray_Allocate();

	DynamicArray_Initialize(furthestPoints1, sizeof(Vector));
	DynamicArray_Initialize(furthestPoints2, sizeof(Vector));

	ConvexHullCollider_GetFurthestPoints(furthestPoints1, convexHull1, (const Vector**)modelOrientedPoints1, relativeMTV);

	//If only a single closest point was found, We have a special case of:
	//	Vertex - Vertex / Edge / Face
	if(furthestPoints1->size == 1)
	{
		//Get the furthest point from the dynamic array
		Vector* furthestPoint = (Vector*)DynamicArray_Index(furthestPoints1, 0);
		//Calculate collision point of vertex - Vertex / edge / face case
		PhysicsManager_DetermineCollisionPointConvexHullVertex(dest, furthestPoint, convexFrame1);
		//Collision point was determined, flag found
		found = 1;
	}

	//If the collision point was not yet determined
	if(!found)
	{
		Vector relativeMTVForObj2;
		Vector_INIT_ON_STACK(relativeMTVForObj2, 3);
		Vector_GetScalarProduct(&relativeMTVForObj2, relativeMTV, -1.0f);
		ConvexHullCollider_GetFurthestPoints(furthestPoints2, convexHull2, (const Vector**)modelOrientedPoints2, &relativeMTVForObj2);


		//If only a single furthest point was found, We have a special case of:
		//	Vertex - Vertex / Edge / Face
		if(furthestPoints2->size == 1)
		{
			//Get the furthest point from the dynamic array
			Vector* furthestPoint = (Vector*)DynamicArray_Index(furthestPoints2, 0);
			//Calculate collision point of Vertex - Vertex / Edge / Face case
			PhysicsManager_DetermineCollisionPointConvexHullVertex(dest, furthestPoint, convexFrame2);
			//Collision point was determined, flag found
			found = 1;
		}

	}


	//If the collision point has still yet to be found
	if(!found)
	{
		//If both objects have 2 furthest points we have a special case of:
		//	Edge - Edge
		if(furthestPoints1->size == 2 && furthestPoints2->size == 2)
		{
			//Caculate collision point of Edge - Edge case
			PhysicsManager_DetermineCollisionPointConvexHullEdge(dest, furthestPoints1, convexFrame1, furthestPoints2, convexFrame2);
			//Collision point wasdetermined, flag found
			found = 1;
		}
	}

	//If th collision point has still yet to be found
	if(!found)
	{
		//It must be an Edge / Face - Face case

		//In this case we must translate all points into worldspace before calling the collision point determination function!
		for(int i = 0; i < furthestPoints1->size; i++)
		{
			Vector* current = (Vector*)DynamicArray_Index(furthestPoints1, i);
			Vector_Increment(current, convexFrame1->position);
		}

		for(int i = 0; i < furthestPoints2->size; i++)
		{
			Vector*current = (Vector*)DynamicArray_Index(furthestPoints2, i);
			Vector_Increment(current, convexFrame2->position);
		}

		//Calculate approximate collision point of general case
		PhysicsManager_DetermineCollisionPointConvexHullFace(dest, furthestPoints1, convexFrame1, furthestPoints2, convexFrame2);
		//Collision point was determined, flag found
		found = 1;
	}

	//Delete arrays of modelSpace oriented collider points
	for(unsigned int i = 0; i < convexHull1->points->size; i++)
	{
		Vector_Free(modelOrientedPoints1[i]);
	}
	free(modelOrientedPoints1);
	for(unsigned int i = 0; i < convexHull2->points->size; i++)
	{
		Vector_Free(modelOrientedPoints2[i]);
	}
	free(modelOrientedPoints2);


	//Delete dynamic arrays of furthest points
	DynamicArray_Free(furthestPoints1);
	DynamicArray_Free(furthestPoints2);
}

///
//Determines the point of contact of a Convex Hull - Convex Hull collision when
//dealing with a Vertex on Vertex / Edge / Face collision case. That is when
//One convex hull has only a single vertex furthest in the direction of a minimum translation vector
//
//Parameters:
//	dest: A pointer to the vector to store the collision point in
//	furthestVertex: A pointer to the vector which registered as furthest in the direction of the MTV
//	frame: A pointer to the frame of reference of the convex hull which the furthestVertex belongs to
static void PhysicsManager_DetermineCollisionPointConvexHullVertex(Vector* dest, const Vector* furthestVertex, const FrameOfReference* frame)
{
	//If only one vertex is furthest in the direction of the relative MTV, this vertex translated into world space
	//Must be the point of collision!
	Vector_Add(dest, furthestVertex, frame->position);
}

///
//Determines the point of contact of a convex hull - convex hull collision when
//dealing with an Edge - Edge collision case. That is when both convex hulls have 
//exactly 2 vertices furthest in the direction of their respective MTVs
//
//Parameters:
//	dest: A pointer to the vector to store the collision point in
//	furthestOnHull1: A pointer to a dynamic array of vectors containing two vertices oriented in modelSpace on convexHull1 furthest in the direction of the respective relative MTV
//	convexFrame1: A pointer to the frame of reference of the convex hull which the vertices furthestOnHull1 belong to
//	furthestOnHull2: A pointer to a dynamic array of vectors containing two vertices oriented in ModelSpace on convexHull2 furthest in the direction of the respective relative MTV
//	convexFrame2: A pointer to the frame of reference of the convex Hull which the vertices furthestOnHull2 belong to
static void PhysicsManager_DetermineCollisionPointConvexHullEdge(Vector* dest, 
																 const DynamicArray* furthestOnHull1, const FrameOfReference* convexFrame1,
																 const DynamicArray* furthestOnHull2, const FrameOfReference* convexFrame2)
{
	//We can represent the two points on each convex hull as a line, then test for the intersection of the two lines
	Vector direction1;	//Will store Direction of line on convexHull1
	Vector direction2;	//Will store Direction of line on convexHull2

	Vector offset1;		//Will store offset of line on convex hull 1 from origin of worldspace
	Vector offset2;		//Will store offset of line on convex hull 2 from origin of worldspace

	//Initialize directions & offsets
	Vector_INIT_ON_STACK(direction1, 3);
	Vector_INIT_ON_STACK(direction2, 3);
	Vector_INIT_ON_STACK(offset1, 3);
	Vector_INIT_ON_STACK(offset2, 3);

	//Determine line directions
	//Const modifier is removed from dynamic arrays but data will not be altered in this function.
	Vector_Subtract(&direction1, (Vector*)DynamicArray_Index((DynamicArray*)furthestOnHull1, 1), (Vector*)DynamicArray_Index((DynamicArray*)furthestOnHull1, 0));
	Vector_Subtract(&direction2, (Vector*)DynamicArray_Index((DynamicArray*)furthestOnHull2, 1), (Vector*)DynamicArray_Index((DynamicArray*)furthestOnHull2, 0));

	//Determine line offsets from orign of worldspace
	//Const modifier is removed from dynamic arrays but data will not be altered in this function.
	Vector_Add(&offset1, convexFrame1->position, (Vector*)DynamicArray_Index((DynamicArray*)furthestOnHull1, 0));
	Vector_Add(&offset2, convexFrame2->position, (Vector*)DynamicArray_Index((DynamicArray*)furthestOnHull2, 0));

	//Solve for t
	//offset1 + t*direction1 = offset2 + t*direction2
	//offset1 - offset2 = t * (direction2 - direction1)
	//offsetFinal = t * directionFinal
	Vector offsetFinal;
	Vector directionFinal;

	Vector_INIT_ON_STACK(offsetFinal, 3);
	Vector_INIT_ON_STACK(directionFinal, 3);

	//offsetFinal = offset1 - offset2
	Vector_Subtract(&offsetFinal, &offset1, &offset2);
	//directionFinal = direction2 - direction1
	Vector_Subtract(&directionFinal, &direction2, &direction1);

	//TODO: Run a test to make sure all solutions for t are equal
	//We now have a system of equations for 3 solutions for t. We should solve for one which has a nonzero directionFinal component
	float t = 0.0f;
	for(int i = 0; i < 3; i++)
	{
		if(directionFinal.components[i] != 0.0f)
		{
			//t = offsetFinal / directionFinal
			t = offsetFinal.components[i] / directionFinal.components[i];
			break;
		}
	}

	//Now we can use either of the equations for the lines to solve for the collision point!
	//dest = offset1 + t * direction1
	Vector_GetScalarProduct(dest, &direction1, t);
	Vector_Increment(dest, &offset1);
}

///
//Determines the point of contact of a convex hull - convex hull collision when
//dealing with an Edge / Face - Face collision case. This is when both convex hulls have
// > 1 vertex furthest in the direction of their respective MTVs AND one convex hull has > 2
//vertices furthest in the direction of it's respective MTV
//
//Parameters:
//	dest: A pointer to he vector to store the collision in
//	furthestOnHull1: A pointer to the dynamic array of vectors containing the vertices belonging to convexHull1 furthest in the direction of it's respective MTV
//	convexFrame1: A pointer to the frame of reference of the convex hull which the vertices from furthestOnHull1 belong
//	furthestOnHull2: A pointer to the dynamic array of vectors containing the vertices belongong to convexHull2 furthest in the direction of it's respective MTV
//	convexFrame2: A pointer to the frame of reference of the convex hull whch the vertices from furthestOnHull2 belong
static void PhysicsManager_DetermineCollisionPointConvexHullFace(Vector* dest,
																 const DynamicArray* furthestOnHull1, const FrameOfReference* convexFrame1,
																 const DynamicArray* furthestOnHull2, const FrameOfReference* convexFrame2)
{

	//Create a linked list of points to find the inner most points
	LinkedList* innerPoints = LinkedList_Allocate();
	LinkedList_Initialize(innerPoints);


	//We must cycle through all points for each axis removing the minimum and maximum bounds each cycle until there remains only 2 points or less.
	LinkedList_Node* iterator = NULL;
	LinkedList_Node* minNode = NULL;
	LinkedList_Node* maxNode = NULL;

	Vector* currentVector = NULL;
	Vector* minVector = NULL;
	Vector* maxVector = NULL;

	for(unsigned int i = 0; i < 3; i++)
	{
		//Add all points translated into worldspace to the list
		for(unsigned int j = 0; j < furthestOnHull1->size; j++)
		{
			//Const modifier on furthestHull1 is removed, but data will not be changed
			LinkedList_Append(innerPoints, DynamicArray_Index((DynamicArray*)furthestOnHull1, j));
		}

		for(unsigned int j = 0; j < furthestOnHull2->size; j++)
		{
			//Const modifier on furthestHull1 is removed, but data will not be changed
			LinkedList_Append(innerPoints, DynamicArray_Index((DynamicArray*)furthestOnHull2, j));
		}

		//Until we have only 2 points left for the ith axis test
		while(innerPoints->size > 2)
		{

			iterator = innerPoints->head;
			minNode = iterator;
			maxNode = iterator;
			for(int j = 0; j < innerPoints->size - 1; j++)
			{
				//Move to next point
				iterator = iterator->next;
				//Get point as vector
				currentVector = (Vector*)iterator->data;
				minVector = (Vector*)minNode->data;
				maxVector = (Vector*)maxNode->data;

				//Compare ith values to max and min
				if(currentVector->components[i] <= minVector->components[i])
				{
					minNode = iterator;
				}
				else if(currentVector->components[i] >= maxVector->components[i])
				{
					maxNode = iterator;
				}
			}
			//In case they are somehow the same, only remove one.
			if(minNode == maxNode)
			{
				LinkedList_RemoveNode(innerPoints, minNode);
			}
			else
			{
				//Each loop remove the max and min from the list
				LinkedList_RemoveNode(innerPoints, minNode);
				LinkedList_RemoveNode(innerPoints, maxNode);
			}

		}

		//Find the average ith value of remaining nodes
		iterator = innerPoints->head;
		while(iterator != NULL)
		{
			Vector* currentVector = (Vector*)iterator->data;
			dest->components[i] += currentVector->components[i];
			iterator = iterator->next;
		}
		dest->components[i] /= (float)innerPoints->size;

		//Clear remaining contents of list for next axis test
		LinkedList_Clear(innerPoints);
	}

	LinkedList_Free(innerPoints);
}

///
//Calculates and imparts the resulting collision impulse from the collision
//
//Parameters:
//	collision: The collision having it's resulting impulses calculated and applied
//	pointOfCollision: Vector representing the point of collision in global space
static void PhysicsManager_ApplyCollisionImpulses(Collision* collision, const Vector** pointsOfCollision)
{
	//Calculation for the numerator of Chris Hecker's collision impulse equation
	//Hardcode coefficients of restitution for now
	float coefficientOfRestitution1 = collision->obj1->body != NULL ? collision->obj1->body->coefficientOfRestitution : 1.0f;
	float coefficientOfRestitution2 = collision->obj2->body != NULL ? collision->obj2->body->coefficientOfRestitution : 1.0f;

	//TODO: Find out if the final coefficient e is the product of the two coefficients of restitution
	float e = coefficientOfRestitution1 * coefficientOfRestitution2;

	Vector radP1;	//Radial vector from CoM of obj1 to point of collision
	Vector radP2;	//Radial vector from CoM of obj2 to point of collision

	Vector_INIT_ON_STACK(radP1, 3);
	Vector_INIT_ON_STACK(radP2, 3);

	Vector_Subtract(&radP1, pointsOfCollision[0], collision->obj1Frame->position);
	Vector_Subtract(&radP2, pointsOfCollision[1], collision->obj2Frame->position);


	Vector velP1;	//total Linear Velocity at point P on obj1
	Vector velP2;	//Total linear velocity at point P on obj2

	Vector_INIT_ON_STACK(velP1, 3);
	Vector_INIT_ON_STACK(velP2, 3);

	if(collision->obj1->body != NULL)
	{
		Vector_CrossProduct(&velP1, collision->obj1->body->angularVelocity, &radP1);
		Vector_Increment(&velP1, collision->obj1->body->velocity);
	}
	else
	{
		Vector_Copy(&velP1, &Vector_ZERO);
	}

	if(collision->obj2->body != NULL)
	{
		Vector_CrossProduct(&velP2, collision->obj2->body->angularVelocity, &radP2);
		Vector_Increment(&velP2, collision->obj2->body->velocity);
	}
	else
	{
		Vector_Copy(&velP2, &Vector_ZERO);
	}

	Vector velAB;	//Relative linear velocity of point P on object A from an observer on Point P on obj B
	Vector_INIT_ON_STACK(velAB, 3);

	Vector_Subtract(&velAB, &velP1, &velP2);

	//Get the magnitude of the relative velocity in the direction of obj1 from the point of contact
	float relNormalVel = Vector_DotProduct(&velAB, collision->minimumTranslationVector);


	//The final numerator of the impulse equation
	float numerator =  (-1 - e) * relNormalVel;

	//Calculation for Denominator of Chris Hecker's impulse equation
	Vector torque1;	//Torque due to collision on obj1
	Vector torque2;	//Torque due to collision on obj2

	Vector_INIT_ON_STACK(torque1, 3);
	Vector_INIT_ON_STACK(torque2, 3);

	Vector_CrossProduct(&torque1, &radP1, collision->minimumTranslationVector);
	Vector_CrossProduct(&torque2, &radP2, collision->minimumTranslationVector);


	Vector velPFromT1;	//Linear velocity of point P on obj1 due to torque of obj1
	Vector velPFromT2;	//Linear velocity of point P on Obj2 due to torque of obj2

	Vector_INIT_ON_STACK(velPFromT1, 3);
	Vector_INIT_ON_STACK(velPFromT2, 3);

	if(collision->obj1->body != NULL && collision->obj1->body->inverseMass != 0.0f)
	{
		Matrix inertiaInWorldSpace;
		Matrix_INIT_ON_STACK(inertiaInWorldSpace, 3, 3);

		RigidBody_CalculateMomentOfInertiaInWorldSpace(&inertiaInWorldSpace, collision->obj1->body);

		Matrix inverseInertia;
		Matrix_INIT_ON_STACK(inverseInertia, 3, 3);
		Matrix_GetInverse(&inverseInertia, &inertiaInWorldSpace);

		//Calculate angular acceleration due to torque
		Matrix_TransformVector(&inverseInertia, &torque1);

		//Determine linear velocity of pont P on obj1 due to angular acceleration of obj1
		Vector_CrossProduct(&velPFromT1, &torque1, &radP1);
	}
	else
	{
		Vector_Copy(&velPFromT1, &Vector_ZERO);
	}

	if(collision->obj2->body != NULL && collision->obj2->body->inverseMass != 0.0f)
	{
		Matrix inertiaInWorldSpace;
		Matrix_INIT_ON_STACK(inertiaInWorldSpace, 3, 3);

		RigidBody_CalculateMomentOfInertiaInWorldSpace(&inertiaInWorldSpace, collision->obj2->body);

		Matrix inverseInertia;
		Matrix_INIT_ON_STACK(inverseInertia, 3, 3);
		Matrix_GetInverse(&inverseInertia, &inertiaInWorldSpace);

		//Calculate angular acceleration due to torque
		Matrix_TransformVector(&inverseInertia, &torque2);

		//Determine linear velocity of pont P on obj1 due to angular acceleration of obj1
		Vector_CrossProduct(&velPFromT2, &torque2, &radP2);
	}
	else
	{
		Vector_Copy(&velPFromT2, &Vector_ZERO);
	}

	//Add the velocities due to torques together
	Vector_Increment(&velPFromT1, &velPFromT2);


	//Get the sum of the inverse masses
	float iMassSum = 0.0f;
	if(collision->obj1->body != NULL)
	{
		iMassSum += collision->obj1->body->inverseMass;
	}

	if(collision->obj2->body != NULL)
	{
		iMassSum += collision->obj2->body->inverseMass;
	}

	//Final calculation for denominator of impulse equation
	float denominator = iMassSum + Vector_DotProduct(&velPFromT1, collision->minimumTranslationVector);

	//Calculate impulse
	float impulse = numerator/denominator;

	Vector impulseVector;
	Vector_INIT_ON_STACK(impulseVector, 3);
	//V1After = V1Before + impulse / M1 * MTV
	Vector_GetScalarProduct(&impulseVector, collision->minimumTranslationVector, impulse);
	//Apply impulse
	if(collision->obj1->body != NULL && collision->obj1->body->inverseMass != 0.0f)
	{
		RigidBody_ApplyImpulse(collision->obj1->body, &impulseVector, &radP1);
	}
	if(collision->obj2->body != NULL && collision->obj2->body->inverseMass != 0.0f)
	{
		//V2After = V2Before - impulse / M2 * MTV
		Vector_Scale(&impulseVector, -1.0f);

		RigidBody_ApplyImpulse(collision->obj2->body, &impulseVector, &radP2);

	}

}

///
//Calculates and applies the frictional forces when two objects slide against each other
//Algorithm is given by:
//	http://en.wikipedia.org/wiki/Collision_response#Impulse-Based_Friction_Model
//
//Parameters:
//	collision: The collision to apply frictional forces to
//	pointsOfCollision: An array of 2 pointers to vectors in 3 space representing the point of collision in global space for obj1 and obj2 respectively
//	staticCoefficient: The static coefficient of friction between the two colliding surfaces
//	dynamicCoefficient: The dynamic coefficient of friction between the two colliding surfaces
static void PhysicsManager_ApplyLinearFrictionalImpulses(Collision* collision, const Vector** pointsOfCollision, const float staticCoefficient, const float dynamicCoefficient)
{
	//Step 1) Find the direction of a tangent vector which is tangential to the surface of collision in the direction of movement / applied forces if there is no tangential movement
	Vector unitTangentVector;
	Vector_INIT_ON_STACK(unitTangentVector, 3);

	Vector relativeVelocity;
	Vector_INIT_ON_STACK(relativeVelocity, 3);

	//Find relative velocity of object2 from observer on object1
	if(collision->obj2->body != NULL)
	{
		Vector_Copy(&relativeVelocity, collision->obj2->body->velocity); 
	}
	if(collision->obj1->body != NULL)
	{
		Vector_Decrement(&relativeVelocity, collision->obj1->body->velocity);
	}

	//Make sure the relative velocity is nonZero
	if(Vector_GetMag(&relativeVelocity) > 0.0f)
	{
		//If there is relative movement between the objects we must figure out the component of this relative velocity which is tangent to the surface
		Vector relativeVelocityPerp;
		Vector_INIT_ON_STACK(relativeVelocityPerp, 3);


		//Project the relative velocity onto the surface normal
		//Vector_GetScalarProduct(&relativeVelocityPerp, &relativeVelocity, Vector_DotProduct(&relativeVelocity, collision->minimumTranslationVector));
		Vector_GetProjection(&relativeVelocityPerp, &relativeVelocity, collision->minimumTranslationVector);

		//VTangential = V - VPerpendicular
		Vector_Subtract(&unitTangentVector, &relativeVelocity, &relativeVelocityPerp);
		Vector_Normalize(&unitTangentVector);
	}

	//if the unit tangent vector is still 0
	if(Vector_GetMag(&unitTangentVector) == 0.0f)
	{
		//We must use the sum of the external forces to compute the unit tangent vector instead of the relative velocity
		Vector cumulativeNetForce;
		Vector_INIT_ON_STACK(cumulativeNetForce, 3);


		if(collision->obj1->body != NULL)
		{
			Vector_Increment(&cumulativeNetForce, collision->obj1->body->previousNetForce);
		}
		if(collision->obj2->body != NULL)
		{
			Vector_Increment(&cumulativeNetForce, collision->obj2->body->previousNetForce);
		}

		//Project the net external force onto the surface normal
		Vector cumNetForcePerp;
		Vector_INIT_ON_STACK(cumNetForcePerp, 3);

		//Vector_GetScalarProduct(&cumNetForcePerp, &cumulativeNetForce, Vector_DotProduct(&cumulativeNetForce, collision->minimumTranslationVector));
		Vector_GetProjection(&cumNetForcePerp, &cumulativeNetForce, collision->minimumTranslationVector);


		//VTangential = V - VPerpendicular
		Vector_Subtract(&unitTangentVector, &cumulativeNetForce, &cumNetForcePerp);
		Vector_Normalize(&unitTangentVector);
	}

	//Step 2) Compute the static and dynamic frictional force magnitudes based off of the magnitude of the
	//component of the reaction impulse of the collision in the direction of the contact normal
	float reactionMag = 0.0f;
	if(collision->obj1->body != NULL && collision->obj1->body->inverseMass != 0.0f && !collision->obj1->body->freezeTranslation)
	{
		reactionMag = fabs(Vector_DotProduct(collision->obj1->body->netImpulse, collision->minimumTranslationVector));
	}
	else
	{
		reactionMag = fabs(Vector_DotProduct(collision->obj2->body->netImpulse, collision->minimumTranslationVector));
	}

	float staticMag = staticCoefficient * reactionMag;
	float dynamicMag = dynamicCoefficient * reactionMag;

	//Step 3) Compute & apply the frictional impulse
	//If none of the relative velocity is in the direction of the tangent vector (The object is not moving) 
	//Or the impulse which would have caused the velocity-- in the direction of the tangent vector is less than the static mag
	float relVelocityTangentialMag = Vector_DotProduct(&relativeVelocity, &unitTangentVector);

	float relImpulseTangentialMag1;
	float relImpulseTangentialMag2;
	if(collision->obj1->body != NULL && collision->obj1->body->inverseMass != 0.0f)
	{
		relImpulseTangentialMag1 = relVelocityTangentialMag / collision->obj1->body->inverseMass;	
		if(relImpulseTangentialMag1 <= staticMag)
		{
			Vector frictionalImpulse;
			Vector_INIT_ON_STACK(frictionalImpulse, 3);

			Vector_GetScalarProduct(&frictionalImpulse, &unitTangentVector, relImpulseTangentialMag1);

			//RigidBody_ApplyImpulse(collision->obj1->body, &frictionalImpulse, pointsOfCollision[0]);
			RigidBody_ApplyImpulse(collision->obj1->body, &frictionalImpulse, &Vector_ZERO);
		}
		else
		{
			Vector frictionalImpulse;
			Vector_INIT_ON_STACK(frictionalImpulse, 3);

			Vector_GetScalarProduct(&frictionalImpulse, &unitTangentVector, dynamicMag);
			//RigidBody_ApplyImpulse(collision->obj1->body, &frictionalImpulse, pointsOfCollision[0]);
			RigidBody_ApplyImpulse(collision->obj1->body, &frictionalImpulse, &Vector_ZERO);
		}
	}
	if(collision->obj2->body != NULL && collision->obj2->body->inverseMass != 0.0f)
	{
		relImpulseTangentialMag2 = relVelocityTangentialMag / collision->obj2->body->inverseMass;

		if(relImpulseTangentialMag2 <= staticMag)
		{
			Vector frictionalImpulse;
			Vector_INIT_ON_STACK(frictionalImpulse, 3);

			Vector_GetScalarProduct(&frictionalImpulse, &unitTangentVector, -relImpulseTangentialMag2);
			//RigidBody_ApplyImpulse(collision->obj2->body, &frictionalImpulse, pointsOfCollision[1]);
			RigidBody_ApplyImpulse(collision->obj2->body, &frictionalImpulse, &Vector_ZERO);
		}
		else
		{
			Vector frictionalImpulse;
			Vector_INIT_ON_STACK(frictionalImpulse, 3);

			Vector_GetScalarProduct(&frictionalImpulse, &unitTangentVector, -dynamicMag);
			//RigidBody_ApplyImpulse(collision->obj2->body, &frictionalImpulse, pointsOfCollision[1]);
			RigidBody_ApplyImpulse(collision->obj2->body, &frictionalImpulse, &Vector_ZERO);
		}
	}


}

///
//Calculates and applies the angular frictional forces when two objects spin against each other
//
//Parameters:
//	collision: The collision to apply frictional torques to
//	staticCoefficient: The static coefficient of friction between the two colliding surfaces
//	dynamicCoefficient: The dynamic coefficient of friction between the two colliding surfaces
static void PhysicsManager_ApplyFrictionalTorques(Collision* collision, const float staticCoefficient, const float dynamicCoefficient)
{
	//Step 1) compute static and dynamic frictional torque magnitudes based off of the magnitude of the
	//component of the reaction instantaneous torque of the collision in the direction of the contact normal
	float reactionMag1 = 0.0f;
	float reactionMag2 = 0.0f;
	if(collision->obj1->body != NULL && collision->obj1->body->inverseMass != 0.0f && !collision->obj1->body->freezeRotation)
	{
		//reactionMag = fabs(Vector_DotProduct(collision->obj1->body->netInstantaneousTorque, collision->minimumTranslationVector));

		Vector instantaneousTorqueCausingAngularAcceleration;
		Vector_INIT_ON_STACK(instantaneousTorqueCausingAngularAcceleration, 3);
		Matrix_GetProductVector(&instantaneousTorqueCausingAngularAcceleration, collision->obj1->body->inertia, collision->obj1->body->angularVelocity);

		Vector_Increment(&instantaneousTorqueCausingAngularAcceleration, collision->obj1->body->netInstantaneousTorque);

		reactionMag1 = fabs(Vector_DotProduct(&instantaneousTorqueCausingAngularAcceleration, collision->minimumTranslationVector));

	}
	else if (collision->obj2->body != NULL && collision->obj2->body->inverseMass != 0.0f && !collision->obj2->body->freezeRotation)
	{
		//reactionMag = fabs(Vector_DotProduct(collision->obj2->body->netInstantaneousTorque, collision->minimumTranslationVector));

		Vector instantaneousTorqueCausingAngularAcceleration;
		Vector_INIT_ON_STACK(instantaneousTorqueCausingAngularAcceleration, 3);
		Matrix_GetProductVector(&instantaneousTorqueCausingAngularAcceleration, collision->obj2->body->inertia, collision->obj2->body->angularVelocity);

		Vector_Increment(&instantaneousTorqueCausingAngularAcceleration, collision->obj2->body->netInstantaneousTorque);

		reactionMag2 = fabs(Vector_DotProduct(&instantaneousTorqueCausingAngularAcceleration, collision->minimumTranslationVector));
	}


	float staticMag1 = staticCoefficient * reactionMag1;
	float dynamicMag1 = dynamicCoefficient * reactionMag1;
	float staticMag2 = staticCoefficient * reactionMag2;
	float dynamicMag2 = dynamicCoefficient * reactionMag2;

	//Step 2) Compute the relative angular velocity between the colliding objects
	Vector relativeAngularVelocity;
	Vector_INIT_ON_STACK(relativeAngularVelocity, 3);
	//Find relative velocity of object2 from observer on object1
	if(collision->obj2->body != NULL)
	{
		Vector_Copy(&relativeAngularVelocity, collision->obj2->body->angularVelocity); 
	}
	if(collision->obj1->body != NULL)
	{
		Vector_Decrement(&relativeAngularVelocity, collision->obj1->body->angularVelocity);
	}

	//Step 3) compute sum of all external torques in the direction of the normal
	Vector cumNetTorques;
	Vector_INIT_ON_STACK(cumNetTorques, 3);

	if(collision->obj1->body != NULL)
	{
		Vector_Increment(&cumNetTorques, collision->obj1->body->previousNetTorque);
	}
	if(collision->obj2->body != NULL)
	{
		Vector_Increment(&cumNetTorques, collision->obj2->body->previousNetTorque);
	}

	Vector_Project(&cumNetTorques, collision->minimumTranslationVector);

	//Step 3) Compute frictional torque
	Vector frictionalTorque;
	Vector_INIT_ON_STACK(frictionalTorque, 3);




	if(collision->obj1->body != NULL)
	{
		float angularInDirectionOfNormal = Vector_DotProduct(collision->obj1->body->angularVelocity, collision->minimumTranslationVector);
		float torqueInDirectionOfNormal = Vector_DotProduct(collision->obj1->body->previousNetTorque, collision->minimumTranslationVector);

		//If the object isn't spinning on the collision face, 
		//and the magnitude of the component of the torque in the direction of the collision face normal is less than the magnitude of static torque due to friction
		if(angularInDirectionOfNormal == 0.0f && fabs(torqueInDirectionOfNormal) <= staticMag1)
		{

			//For OBJ1
			//We can base frictional torque off of the current angular velocity to negate it (Nothing should be moving yet)
			Matrix_GetProductVector(&frictionalTorque, collision->obj1->body->inertia, &relativeAngularVelocity);
			Vector_GetScalarProduct(&frictionalTorque, collision->minimumTranslationVector, -1.0f * Vector_DotProduct(&frictionalTorque, collision->minimumTranslationVector));

			RigidBody_ApplyInstantaneousTorque(collision->obj1->body, &frictionalTorque);

		}
		else
		{
			//Find an axis opposite the direction of the current angular velocity
			Vector axis;
			Vector_INIT_ON_STACK(axis, 3);

			if(Vector_GetMag(collision->obj2->body->angularVelocity) > 0.0f)
				Vector_GetProjection(&axis, collision->obj2->body->angularVelocity, collision->minimumTranslationVector);
			else
				Vector_GetProjection(&axis, collision->obj2->body->previousNetTorque, collision->minimumTranslationVector);

			Vector_Normalize(&axis);

			//Torque will be in the direction of the collision face normal with a magnitude of -1.0 * dynamic mag
			Vector_GetScalarProduct(&frictionalTorque, &axis, -1.0f * dynamicMag2);

			RigidBody_ApplyInstantaneousTorque(collision->obj1->body, &frictionalTorque);
		}
	}


	//OBJ2
	if(collision->obj2->body != NULL)
	{
		float angularInDirectionOfNormal = Vector_DotProduct(collision->obj2->body->angularVelocity, collision->minimumTranslationVector);
		float torqueInDirectionOfNormal = Vector_DotProduct(collision->obj2->body->previousNetTorque, collision->minimumTranslationVector);
		//If the object isn't spinning on the collision face, 
		//and the magnitude of the component of the torque in the direction of the collision face normal is less than the magnitude of static torque due to friction
		if(angularInDirectionOfNormal == 0.0f && fabs(torqueInDirectionOfNormal) <= staticMag2)
		{

			//We can base frictional torque off of the current angular velocity to negate it (Nothing should be moving yet)
			Matrix_GetProductVector(&frictionalTorque, collision->obj2->body->inertia, &relativeAngularVelocity);
			Vector_GetScalarProduct(&frictionalTorque, collision->minimumTranslationVector, -1.0f * Vector_DotProduct(&frictionalTorque, collision->minimumTranslationVector));

			RigidBody_ApplyInstantaneousTorque(collision->obj2->body, &frictionalTorque);
		}
		else
		{
			//Find an axis opposite the direction of the current angular velocity
			Vector axis;
			Vector_INIT_ON_STACK(axis, 3);

			if(Vector_GetMag(collision->obj2->body->angularVelocity) > 0.0f)
				Vector_GetProjection(&axis, collision->obj2->body->angularVelocity, collision->minimumTranslationVector);
			else
				Vector_GetProjection(&axis, collision->obj2->body->previousNetTorque, collision->minimumTranslationVector);

			Vector_Normalize(&axis);

			Vector_GetScalarProduct(&frictionalTorque, &axis, -1.0f * dynamicMag2);

			RigidBody_ApplyInstantaneousTorque(collision->obj2->body, &frictionalTorque);

		}

	}

}