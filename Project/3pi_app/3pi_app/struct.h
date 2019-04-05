/*
 * CFile1.c
 *
 * Created: 4/3/2019 11:38:52 PM
 *  Author: Alex
 */ 
#ifndef STRUCT_H_   /* Include guard */
#define STRUCT_H_
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

struct vertex
{
	int x;
	int y;
	int used;
	struct vertex* prev;
	struct vertex* next;
	int passable;
	double distToGoal;
	double distFromStart;
};

// A structure to represent a queue
struct Queue
{
	int front, rear, size;
	unsigned capacity;
	struct vertex* array;
};

// function to create a queue of given capacity.
// It initializes size of queue as 0
struct Queue* createQueue(unsigned capacity)
{
	struct Queue* queue = (struct Queue*) malloc(sizeof(struct Queue));
	queue->capacity = capacity;
	queue->front = queue->size = 0;
	queue->rear = capacity - 1;  // This is important, see the enqueue
	queue->array = (struct vertex*) malloc(queue->capacity * sizeof(struct vertex));
	return queue;
}

// Queue is full when size becomes equal to the capacity
int isFull(struct Queue* queue)
{  return (queue->size == queue->capacity);  }

// Queue is empty when size is 0
int isEmpty(struct Queue* queue)
{  return (queue->size == 0); }

// Function to add an item to the queue.
// It changes rear and size
void enqueue(struct Queue* queue,struct vertex item)
{
	if (isFull(queue))
	return;
	
	queue->rear = (queue->rear + 1)%queue->capacity;
	queue->array[queue->rear] = item;
	queue->size = queue->size + 1;
	printf("%d enqueued to queue\n", item);
}

// Function to remove an item from queue.
// It changes front and size
struct vertex* dequeue(struct Queue* queue)
{
	if (isEmpty(queue))
	return NULL;
	struct vertex item = queue->array[queue->front];
	queue->front = (queue->front + 1)%queue->capacity;
	queue->size = queue->size - 1;
	return &(item);
}

// Function to get front of queue
struct vertex* front(struct Queue* queue)
{
	if (isEmpty(queue))
	return NULL;
	return &(queue->array[queue->front]);
}

// Function to get rear of queue
struct vertex* rear(struct Queue* queue)
{
	if (isEmpty(queue))
	return NULL;
	return &(queue->array[queue->rear]);
}

#endif 
//https://www.geeksforgeeks.org/queue-set-1introduction-and-array-implementation/