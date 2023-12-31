#include "main.h"
#include "rigidBody.h"

void initRigidBall(RigidBall *ball, double radius, double mass){
	ball->state.xPos = 0.010;
	ball->state.yPos = 0;
	ball->state.theta = 0;
	
	ball->state.xSpe = 0;
	ball->state.ySpe = 0;
	ball->state.aSpe = 0;

	ball->state.mass = mass;
	ball->radius = radius;
}

void printRigidBallState(RigidBall *ball){
	printf("ID: %d\tX: %.3f\tY: %.3f\tX Speed:%.2f\tY Speed:%.2f\n", ball->state.id, ball->state.xPos, ball->state.yPos, ball->state.xSpe, ball->state.ySpe);
}

void drawRigidBall(SDL_Renderer *ren, RigidBall *ball, unsigned int winH, unsigned int winW){
	SDL_Rect dispRect;
	dispRect.x = winW/2 - ball->state.xPos * PIXELS_PER_METER - 25;
	dispRect.y = winH/2 - ball->state.yPos * PIXELS_PER_METER - 25;
	dispRect.h = 50; //ball->radius * PIXELS_PER_METER ;
	dispRect.w = 50; //ball->radius * PIXELS_PER_METER ;
	
	SDL_SetRenderDrawColor(ren, 0xFF, 0x00, 0x00, 0xFF);
	SDL_RenderDrawRect(ren, &dispRect);
	SDL_RenderDrawLine(ren,  winW/2, winH/2, dispRect.x+25, dispRect.y+25);
}

//Evaluates function at simTime, increasing dT time steps, takes the derivative of the current value and the previous value
//Rk4 implementation
double odeSolve(double fun, double funDer, double funDerDer, double dT, double *simTime){
	double k1, k2, k3, k4;

	k1 = funDer;
	k2 = funDer + funDerDer * (dT/2);
	k3 = funDer + funDerDer * (dT/2);
	k4 = funDer + funDerDer * dT;

	*simTime += dT;

	return fun + (dT/6) * (k1 + 2*k2 + 2*k3 + k4);
}

void stepTime(RigidState *state, double forceMat[][MAX_OBJS][DIMENSIONS], double dT, int objCount, double *simTime){
/*
	state->xPos += state->xSpe * dT;
	state->yPos += state->ySpe * dT;
*/


	double xAcc = getTotForce(forceMat, state->id, objCount, 0) / state->mass;
	double yAcc = getTotForce(forceMat, state->id, objCount, 1) / state->mass;
	
//	printf("\t%.2fX\t%.2fY\n", xAcc, yAcc);	

	state->xSpe += xAcc * dT;
	state->ySpe += yAcc * dT;

	state->xPos = odeSolve(state->xPos, state->xSpe, xAcc, dT, simTime);
	state->yPos = odeSolve(state->yPos, state->ySpe, yAcc, dT, simTime);
	
}

