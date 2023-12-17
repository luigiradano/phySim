#include "rigidBody.h"

void initRigidBall(RigidBall *ball, double radius, double mass){
	ball->state.xPos = 0;
	ball->state.yPos = 0;
	ball->state.theta = 0;
	
	ball->state.xSpe = 0;
	ball->state.ySpe = 0;
	ball->state.aSpe = 0;

	ball->state.mass = mass;
	ball->radius = radius;
}


