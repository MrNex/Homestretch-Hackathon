#include "ObjectManager.h"
#include <stdio.h>

///
//Initializes the Object manager
void ObjectManager_Initialize(void)
{
	objectBuffer = ObjectManager_AllocateBuffer();
	ObjectManager_InitializeBuffer(objectBuffer);
}

///
//Frees all internal data managed by the Object Manager
void ObjectManager_Free(void)
{
	ObjectManager_FreeBuffer(objectBuffer);
}

///
//Gets the internal Object Buffer being managed by the Object Manager
//
//Returns:
//	Internal Object Buffer of Object Manager
ObjectBuffer ObjectManager_GetObjectBuffer(void)
{
	return *objectBuffer;
}

///
//Updates the internal state of all contained objects.
void ObjectManager_Update(void)
{
	struct LinkedList_Node* current = objectBuffer->gameObjects->head;
	struct LinkedList_Node* next = NULL;
	while (current != NULL)
	{
		next = current->next;
		GObject* gameObj = (GObject*)(current->data);

		GObject_Update(gameObj);

		current = next;
	}
}

///
//Updates the internal state of the OctTree
void ObjectManager_UpdateOctTree(void)
{
	OctTree_Update(objectBuffer->octTree, objectBuffer->gameObjects);
}

///
//Adds an object to collection of objects managed by the Object Manager
//
//Parameters:
//	obj: The object to add
void ObjectManager_AddObject(GObject* obj)
{
	LinkedList_Append(objectBuffer->gameObjects, obj);
	if(obj->collider != NULL)
	{
		//Add the object
		OctTree_AddAndLog(objectBuffer->octTree, obj);
	}
}

///
//Removes an object from the collection of objects managed by the Object Manager
//Upon removal the Object Manager will NOT deallocate memory being used by the object!!!
//
//Parameters:
//	obj: The object to remove
void ObjectManager_RemoveObject(GObject* obj)
{
	LinkedList_RemoveValue(objectBuffer->gameObjects, obj);
	if(obj->collider != NULL)
	{
		OctTree_RemoveAndUnLog(objectBuffer->octTree, obj);
	}
}

///
//Removes an object from the collection of objects being managed by the object manager.
//Upon removal the object manager will deallocate memory being used by the object.
//
//Parameters:
//	obj: the object to remove and free
void ObjectManager_DeleteObject(GObject* obj)
{
	ObjectManager_RemoveObject(obj);
	GObject_Free(obj);
}


///
//Allocates a new Object buffer
//
//Returns:
//	Returns a pointer to a newly allocated Object Buffer
static ObjectBuffer* ObjectManager_AllocateBuffer(void)
{
	ObjectBuffer* buffer = (ObjectBuffer*)malloc(sizeof(ObjectBuffer));
	return buffer;
}

///
//Initializes an Object Buffer
//
//Parameters:
//	buffer: The object buffer to initialize
static void ObjectManager_InitializeBuffer(ObjectBuffer* buffer)
{
	buffer->gameObjects = LinkedList_Allocate();
	LinkedList_Initialize(buffer->gameObjects);

	buffer->octTree = OctTree_Allocate();
	OctTree_Initialize(buffer->octTree, -50.0f, 50.0f, -50.0f, 50.0f, -50.0f, 50.0f);
}

///
//Frees resources being used by an Object Buffer
//
//Parameters:
//	buffer: The object buffer to free
static void ObjectManager_FreeBuffer(ObjectBuffer* buffer)
{
	//Free the oct tree
	OctTree_Free(buffer->octTree);

	//Delete all Objects being held in the object buffer
	struct LinkedList_Node* current = buffer->gameObjects->head;

	while (current != NULL)
	{
		GObject* data = (GObject*)current->data;
		GObject_Free(data);
		current = current->next;
	}

	//Now remove the list
	LinkedList_Free(buffer->gameObjects);

}