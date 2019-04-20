/*
 * MovingAndPathLogic.c
 *
 * Created: 4/3/2019 10:14:25 PM
 *  Author: Alex
 */ 
#include "struct.h"
struct vertex Matrix [100][100];

void SaveEdge(struct vertex * child, struct vertex * parent){
	child->prev = parent;
	parent->next = child;
}

void RemoveEdge(struct vertex * child, struct vertex * parent){
	child->prev = NULL;
	parent->next = NULL;
}

void PrintPath(struct vertex* start){
	struct vertex* v = start;
	do{
		printf("x:%d,y:%d",v->x, v->y);
		if(v->next != NULL){
			v = &Matrix[v->next->x][v->next->y];
		}
	}while(v->next != NULL);
}

struct vertex ChooseNextStep(struct vertex* v){
	
	struct vertex list [8] = {
		Matrix[(v->x)-1][v->y],//west
		Matrix[(v->x)-1][(v->y)-1],//northwest
		Matrix[(v->x)+1][(v->y)],//east
		Matrix[(v->x)+1][(v->y)-1],//northeast	
		Matrix[(v->x)][v->y+1],//south
		Matrix[(v->x)-1][(v->y)+1],//southwest
		Matrix[(v->x)+1][(v->y)+1],//southeast
		Matrix[(v->x)][(v->y)-1],//northwest
	};
	struct vertex next = list[0];
	//if(next == goal) success;
	for(int i = 1;i<8;i++){
		if(!next.passable || next.used || 
		list[i].passable == 1 
		&& (list[i].distFromStart + list[i].distToGoal < next.distToGoal + next.distFromStart) 
		&& list[i].used == 0)
		{
			next = list[i];
		}
	}
	return next;
}