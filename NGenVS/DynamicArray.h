#ifndef DYNAMIC_ARRAY
#define DYNAMIC_ARRAY

typedef struct DynamicArray {
	unsigned int capacity;	//Total available slots
	unsigned int size;		//Current size
	float growthRate;		//Rate array will grow
	unsigned int dataSize;	//Size of data
	void* data;				//Array of data

} DynamicArray;

///
//Allocates a Dynamic Array
//
//Returns:
//	Pointer to a newly allocated dynamic array
DynamicArray* DynamicArray_Allocate();

///
//Initializes a Dynamic Array
//
//Parameters:
//	arr: Dynamic array to initialize
void DynamicArray_Initialize(DynamicArray* arr, unsigned int dataSize);

///
//Frees resources being used by a dynamic array
//
//Parameters:
//	arr: The dynamic array to free
void DynamicArray_Free(DynamicArray* arr);

//Internal members
static int defaultCapacity = 8;
static float defaultGrowth = 2.0f;
//Internal functions
static void DynamicArray_Grow(DynamicArray* arr);

//Functions

///
//Appends a dynamic array with data
//
//Parameters:
//	arr: Dynamic array to append
//	data: Data to append to array
void DynamicArray_Append(DynamicArray* arr, void* data);

///
//Indexes a dynamic array
//
//Parameters:
//	arr: The dynamic array to index
//	index: The index to get
//
//Returns:
//	Pointer to data at the index of arr
void* DynamicArray_Index(DynamicArray* arr, unsigned int index); 

///
//Clears a dynamic array setting all indices & size to 0
//Does not lessen capacity
//
//Parameters:
//	arr: Pointer to dynamic array being cleared
void DynamicArray_Clear(DynamicArray* arr);

///
//Removes an element from the dynamic array.
//Then copies all data back one space to ensure no gaps in data exist.
//
//Parameters:
//	arr: A pointer to the dynamic array to remove an element from
//	index: The index of the element to remove
void DynamicArray_Remove(DynamicArray* arr, const unsigned int index);

///
//Removes a specified data from the dynamic array
//Then copies all data back one space to ensure no gaps in data exist
//
//Parameters:
//	arr: A pointer to the dynamic array to remove data from
//	data: The data to remove
void DynamicArray_RemoveData(DynamicArray* arr, void* data);

///
//Determines if data is already contained in the array
//Searches the entire array
//
//Parameters:
//	arr: The array to search for the data
//	data: The data being searched for
//
//Returns:
//	0 if the data is not contained
//	1 if the data is contained
unsigned char DynamicArray_Contains(DynamicArray* arr, void* data);

///
//Determines if data is already contained in the first n elements of the array
//
//Parameters:
//	arr: The array to search for the data
//	data: The data being searched for
//	n: The number of elements to search from the start of the array
//
//Returns:
//	0 if the data is not contained
//	1 if the data is contained
unsigned char DynamicArray_ContainsWithin(DynamicArray* arr, void* data, unsigned int n);

#endif