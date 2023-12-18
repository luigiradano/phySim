#include "main.h"
#include "rigidBody.h"

#define PIXELS_PER_METER 10

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
	dispRect.x = winW/2 - ball->state.xPos * PIXELS_PER_METER;
	dispRect.y = winH/2 - ball->state.yPos * PIXELS_PER_METER;
	dispRect.h = ball->radius * PIXELS_PER_METER;
	dispRect.w = ball->radius * PIXELS_PER_METER;
	
	SDL_SetRenderDrawColor(ren, 0xFF, 0x00, 0x00, 0xFF);
	SDL_RenderDrawRect(ren, &dispRect);
}

void odeSolve(RigidState *state, double forceMat[][MAX_OBJS][DIMENSIONS], double dT, int objCount){
	state->xPos += state->xSpe * dT;
	state->yPos += state->ySpe * dT;
	
	double xAcc = getTotForce(forceMat, state->id, objCount, 0) / state->mass;
	double yAcc = getTotForce(forceMat, state->id, objCount, 1) / state->mass;
	
	printf("\t%.2fX\t%.2fY\n", xAcc, yAcc);	
	state->xSpe += xAcc * dT;
	state->ySpe += yAcc * dT;

}
