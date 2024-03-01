#include "main.h"
#include "rigidBody.h"

#define TRAJ_POINTS 1500

RigidState *globState[MAX_OBJS];
uint32_t stateIndex = 0; //Will hold the last state that was added, in this way we can keep track of where to push new states

Constraint *globConst[MAX_OBJS];
uint32_t constIndex = 0;

//------------------------------------------- Constraint declarations ---------------------------
uint32_t jacobIndex = 0;

//ToDo: currently hard coded for 4 dimensional jacobians
void fillMatrices(double forceMat[][MAX_OBJS][DIMENSIONS], Constraint *cons, RigidState *states[], Matrix *force, Matrix *velocity, Matrix *mass, int currIndex){
	//Fill force
	setElement(force, 		0,   			currIndex, 		forceMat[EXTERNAL_FORCE][cons->stateAIndex][X_DIM]);
	setElement(force, 		1,   			currIndex, 		forceMat[EXTERNAL_FORCE][cons->stateAIndex][Y_DIM]);
	setElement(force, 		2,   			currIndex, 		forceMat[EXTERNAL_FORCE][cons->stateBIndex][X_DIM]);
	setElement(force, 		3,   			currIndex, 		forceMat[EXTERNAL_FORCE][cons->stateBIndex][Y_DIM]);
	//Fill velocity
	setElement(velocity, 	0,   			currIndex, 		states[cons->stateAIndex]->xSpe);
	setElement(velocity, 	1,   			currIndex, 		states[cons->stateAIndex]->ySpe);
	setElement(velocity, 	2,   			currIndex, 		states[cons->stateBIndex]->xSpe);
	setElement(velocity, 	3,   			currIndex, 		states[cons->stateBIndex]->ySpe);
	//Fill mass
	setElement(mass, 		0,   			0, 				states[cons->stateAIndex]->mass);
	setElement(mass, 		1,   			1, 				states[cons->stateAIndex]->mass);
	setElement(mass, 		2,   			2, 				states[cons->stateBIndex]->mass);
	setElement(mass, 		3,   			3, 				states[cons->stateBIndex]->mass);
}

void updateForcesNull(double forceMat[][MAX_OBJS][DIMENSIONS], Constraint *cons){
	return;
}

void updateCircumForces(double forceMat[][MAX_OBJS][DIMENSIONS], Constraint *cons){
	forceMat[EXTERNAL_FORCE][cons->stateAIndex][X_DIM] -= getTotForce(forceMat, cons->stateBIndex, objCount, X_DIM);
	forceMat[EXTERNAL_FORCE][cons->stateAIndex][Y_DIM] -= forceMat[CONSTRAINT_FORCE][cons->stateBIndex][Y_DIM];
}

void updateCircumJacobains(Matrix *jacob, Matrix *jacob2, Matrix *corrector, int currIndex, Constraint *cons, RigidState *states[MAX_OBJS]){
    double holder[CIRC_JAC_DIMS];

    jacobIndex = currIndex;

    circumJacobian(states[cons->stateAIndex]->xPos, states[cons->stateAIndex]->yPos, states[cons->stateBIndex]->xPos, states[cons->stateBIndex]->yPos, holder);
    vec2row(holder, jacob, jacobIndex, CIRC_JAC_DIMS, 0);

    circumJacobian2(states[cons->stateAIndex]->xSpe, states[cons->stateAIndex]->ySpe, states[cons->stateBIndex]->xSpe, states[cons->stateBIndex]->ySpe, holder);
    vec2row(holder, jacob2, jacobIndex, CIRC_JAC_DIMS, 0);

    double devi = circumResisdual(states[cons->stateAIndex]->xPos, states[cons->stateAIndex]->yPos, states[cons->stateBIndex]->xPos, states[cons->stateBIndex]->yPos, cons->value);
    setElement(corrector, jacobIndex, 0, devi);

    jacobIndex++;
	
}

void updateHingeJacobians(Matrix *jacob, Matrix *jacob2, Matrix *corrector, int currIndex, Constraint *cons, RigidState *states[MAX_OBJS]){
	double holder[HINGE_JAC_DIMS];
	
    hingeJacobian(states[cons->stateAIndex]->xPos, states[cons->stateAIndex]->yPos, states[cons->stateBIndex]->xPos, states[cons->stateBIndex]->yPos, holder);
    vec2row(holder, jacob, currIndex, HINGE_JAC_DIMS, 0);

    hingeJacobian2(states[cons->stateAIndex]->xSpe, states[cons->stateAIndex]->ySpe, states[cons->stateBIndex]->xSpe, states[cons->stateBIndex]->ySpe, holder);
    vec2row(holder, jacob2, currIndex, HINGE_JAC_DIMS, 0);

    double devi = hingeResisdual(states[cons->stateAIndex]->xPos, states[cons->stateAIndex]->yPos, states[cons->stateBIndex]->xPos, states[cons->stateBIndex]->yPos);
    setElement(corrector, currIndex, 0, devi);

}

void initBeamConstraint(Constraint *constraint, uint32_t stateAInd, uint32_t stateBInd, double radius){
    constraint->stateAIndex = stateAInd;
    constraint->stateBIndex = stateBInd;

    constraint->value = radius;
    constraint->valType = RADIUS;

    constraint->updateJacobians = updateCircumJacobains;
    constraint->propagateForces = updateForcesNull;

}

/*
	@brief Initializes a constraint to behave as a hinge (fixes state B to the position of state A)
*/
void initHingeConstraint(Constraint *constraint, uint32_t stateAInd, uint32_t stateBInd){
    constraint->stateAIndex = stateAInd;
    constraint->stateBIndex = stateBInd;

	constraint->value = 0;
    constraint->valType = RADIUS;

    constraint->updateJacobians = updateHingeJacobians;
    constraint->propagateForces = updateCircumForces;

}   
//------------------------------------------- Initializations declarations ---------------------------

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

bool initRigidBeam(RigidBeam *beam, double lenght, double mass, double thickness){

	if(stateIndex + 2 > MAX_OBJS)
		return ERROR;

	beam->stateA.xPos = 1.000;
	beam->stateA.xSpe = 0.000;
	beam->stateA.yPos = 0.000;
	beam->stateA.ySpe = 0.000;
	beam->stateA.theta= 0.000;
	beam->stateA.aSpe = 0.000;
	beam->stateA.id = stateIndex;
	beam->stateA.fixed = 0;

	globState[stateIndex] = &beam->stateA;
	stateIndex++;

	beam->stateB.xPos = 1+lenght;
	beam->stateB.xSpe = 0.000;
	beam->stateB.yPos = 0.000;
	beam->stateB.ySpe = 0.000;
	beam->stateB.theta= 0.000;
	beam->stateB.aSpe = 0.000;
	beam->stateB.id = stateIndex;
	beam->stateB.fixed = 0;

	globState[stateIndex] = &beam->stateB;
	stateIndex++;

	beam->stateA.mass = mass/2.0f;
	beam->stateB.mass = mass/2.0f;

	beam->lenght = lenght;
	beam->thickness = thickness;

	initBeamConstraint(&beam->constraint, beam->stateA.id, beam->stateB.id, lenght);
	registerConstraint(&beam->constraint);

	return NO_ERROR;
}


//------------------------------------------- Helpers ------------------------------------------- 

bool registerConstraint(Constraint *cons){
	if(constIndex > MAX_OBJS)
		return ERROR;
	globConst[constIndex] = cons;
	constIndex ++;
	return NO_ERROR;
}

bool registerState(RigidState *state){
	if(stateIndex > MAX_OBJS)
		return ERROR;
	globState[stateIndex] = state;
	state->id = stateIndex;
	stateIndex ++;
	return NO_ERROR;
}


void printState(RigidState *state, double forceMat[][MAX_OBJS][DIMENSIONS]){
	printf("ID: %d\tX: %.3f\tY: %.3f\tX Speed:%.2f\tY Speed:%.2f\t Force X/Y: %.3f/%.3f\n", state->id, state->xPos, state->yPos, state->xSpe, state->ySpe,getTotForce(forceMat, state->id, objCount, X_DIM), getTotForce(forceMat, state->id, objCount, Y_DIM));
}

double getKinEne(RigidState *state){
	//E_k = 0.5 * m * v^2
	//v^2 = vx^2 + vy^2
	return 0.5 * state->mass * (pow(state->xSpe, 2) + pow(state->ySpe, 2));
}

double getPotEne(RigidState *state, double forceMat[][MAX_OBJS][DIMENSIONS]){
	//E_k = m * g * h
	return -1 * state->yPos * forceMat[EXTERNAL_FORCE][state->id][Y_DIM]; 
}

void printDistance(RigidState *A, RigidState *B){
	double distance = sqrt((A->xPos-B->xPos)*(A->xPos-B->xPos) + (A->yPos-B->yPos)*(A->yPos-B->yPos));
	printf("Distance: %.3f\n", distance);
}
//------------------------------------------- Graphics stuff ------------------------------------------- 

void drawRigidBeam(SDL_Renderer *ren, RigidBeam *beam, unsigned int winH, unsigned int winW){
	SDL_Point start;
	SDL_Point end;

	start.x = beam->stateA.xPos * PIXELS_PER_METER;
	start.y = beam->stateA.yPos * PIXELS_PER_METER;

	end.x = beam->stateB.xPos * PIXELS_PER_METER;
	end.y = beam->stateB.yPos * PIXELS_PER_METER;

	SDL_SetRenderDrawColor(ren, 0xFF, 0x00, 0x00, 0xFF);
	SDL_RenderDrawLine(ren, start.x, start.y, end.x, end.y);
}

void drawRigidBall(SDL_Renderer *ren, RigidBall *ball, unsigned int winH, unsigned int winW, bool drawOriginLink){
	SDL_Rect dispRect;
	dispRect.x = winW/2 - ball->state.xPos * PIXELS_PER_METER - BALL_SIZE/2 + X_OFFSET;
	dispRect.y = winH/2 - ball->state.yPos * PIXELS_PER_METER - BALL_SIZE/2 + Y_OFFSET;
	dispRect.h = BALL_SIZE; //ball->radius * PIXELS_PER_METER ;
	dispRect.w = BALL_SIZE; //ball->radius * PIXELS_PER_METER ;
	
	SDL_SetRenderDrawColor(ren, 0xFF, 0x00, 0x00, 0xFF);
	SDL_RenderFillRect(ren, &dispRect);
	
	if(drawOriginLink)
		SDL_RenderDrawLine(ren,  winW/2+ X_OFFSET, winH/2+ Y_OFFSET, dispRect.x + BALL_SIZE/2, dispRect.y + BALL_SIZE/2);
}

/*
	@brief Draws a link between ball and ball2
*/
void drawLink(SDL_Renderer *ren, RigidState *ball, RigidState *ball2, unsigned int winH, unsigned int winW){
	
	int x1 = winW/2 - ball->xPos * PIXELS_PER_METER + X_OFFSET;
	int y1 = winH/2 - ball->yPos * PIXELS_PER_METER + Y_OFFSET;
	
	int x2 = winW/2 - ball2->xPos * PIXELS_PER_METER + X_OFFSET;
	int y2 = winH/2 - ball2->yPos * PIXELS_PER_METER + Y_OFFSET;
	
	SDL_SetRenderDrawColor(ren, 0x00, 0x00, 0xFF, 0xFF);
	SDL_RenderDrawLine(ren,  x1, y1, x2, y2);
}

/*
	@brief Fills the texture with black
*/
void TextureClearRectangle(int width, int height, uint32_t *pixels)
{	
	uint32_t i, maxIndex = width * height;
    for(i = 0; i < maxIndex; i ++){
		pixels[i] = 0;
	}
}

/*
	@brief Draws a rectangle on a texture
*/
void TextureDrawRectangle(int x0, int y0, int width, int height, uint32_t *pixels, uint32_t rowLen, uint32_t color)
{
    for (int x = x0; x < (x0 + width); x++) {
        for (int y = y0; y < (y0 + height); y++) {
            
			pixels[ x + y * rowLen] = color;
        }
    }
}

/*
	@brief Draws a circle on a texture by utilizing the pixels array directly, rowLen has to be the size of one row in bytes
*/
void TextureDrawCircle(int x0, int y0, float radius, uint32_t *pixels, uint32_t rowLen, uint32_t color)
{
   for (int w = 0; w < radius * 2; w++)
    {
        for (int h = 0; h < radius * 2; h++)
        {
            int dx = radius - w; // horizontal offset
            int dy = radius - h; // vertical offset
            if ((dx*dx + dy*dy) <= (radius * radius))
            {
				pixels[x0 + dx + (y0 + dy) * rowLen] = color;

            }
        }
    }
}
/*
	@brief Takes the current display position of each point along the ball trajectory points
*/
SDL_PointColor trajectory[TRAJ_POINTS];
uint32_t trajIndex = 0;
uint8_t r = 0, g = 255, b = 0, selected = 0;
void drawTraj(SDL_Renderer *ren, RigidState *state, SDL_Texture *trajHandle){
	
	uint32_t *pixels;
	int rowLen; //Each pixel takes 4 bytes
	uint32_t w, h, i;	

	SDL_LockTexture(trajHandle, NULL, (void**) &pixels, &rowLen);
	SDL_QueryTexture(trajHandle, NULL, NULL, &w, &h);
	
	int x1 = w/2 - state->xPos * PIXELS_PER_METER + X_OFFSET;
	int y1 = h/2 - state->yPos * PIXELS_PER_METER + Y_OFFSET;
	
	rowLen /= 4; //Each pixel is 4 bytes

	TextureClearRectangle(w,h,pixels); //Clear whole bg texture

	float radius = 4.5 - sqrt(pow(state->xSpe,2) + pow(state->ySpe,2)) * 1.3 ;
	if(radius < 2)
	 	radius = 2;

	if(!(x1-radius < 0 || y1-radius < 0 || x1+radius > w || y1+radius > h)){

		TextureDrawCircle(x1, y1, radius, pixels, rowLen, 0xFF20d687);

		trajectory[trajIndex].x = x1;
		trajectory[trajIndex].y = y1;
		trajectory[trajIndex].radius = radius;
		trajectory[trajIndex].color = 0xFF20d687; //Always set first byte to 0xFF, it doesn't affect color, used to detect if pixel was updated

		if(trajIndex < TRAJ_POINTS)
			trajIndex ++;
		else
			trajIndex = 0;

	}

	//Update old pixels
	for(i = 0; i < TRAJ_POINTS; i++){
		
		if(trajectory[i].radius >= 0.01){
			trajectory[i].radius -= 4.0f/TRAJ_POINTS;
			TextureDrawCircle(trajectory[i].x, trajectory[i].y, trajectory[i].radius, pixels, rowLen, trajectory[i].color);
		}

	}

	SDL_UnlockTexture(trajHandle);
	
	SDL_RenderCopy(ren, trajHandle, NULL, NULL);

}


//------------------------------------------- ODE Solver ------------------------------------------- 

/*
	Enum establishing the modes of stepTime since we want to update the position every 4 iterations
*/
enum{
	GET_K1,
	GET_K2,
	GET_K3,
	GET_K4,
	COMPUTE
};

uint8_t stepMode[MAX_OBJS][DIMENSIONS];
double k[4][MAX_OBJS][DIMENSIONS], kD[4][MAX_OBJS][DIMENSIONS];
double posAcc[MAX_OBJS][DIMENSIONS]; //Holds the positions, used to update the value in between stepMode changes
double posIni[MAX_OBJS][DIMENSIONS]; //Holds the initial positions, used to compute the final position
double speAcc[MAX_OBJS][DIMENSIONS]; //Holds the speeds, used to update the value in between stepMode changes
double speIni[MAX_OBJS][DIMENSIONS]; //Holds the initial speeds, used to compute the final position

/*
	Needed for the intial states required by the RK4 impelementation
*/
void setInitials(RigidState *states[], int objCount){
	uint32_t i, j;
	for(i = 0; i < objCount; i ++){
		
		posIni[i][X_DIM] = states[i]->xPos;
		posIni[i][Y_DIM] = states[i]->yPos;
		speIni[i][X_DIM] = states[i]->xSpe;
		speIni[i][Y_DIM] = states[i]->ySpe;
		
	}
}

/*
	RK4 Implementation responsible for simulating the speed of a single state
*/
void stepSingleTime(RigidState *state, double forceMat[][MAX_OBJS][DIMENSIONS], double dT, int objCount, uint16_t dimension){
	
	double acc = getTotForce(forceMat, state->id, objCount, dimension) / state->mass;

	double *speed, *pos;


	//Step the mode only when X coordinate is recieved
	if(stepMode[state->id][dimension] < COMPUTE)
		stepMode[state->id][dimension] ++;
	else
		stepMode[state->id][dimension] = 0;

	
	switch(dimension){
		case X_DIM:
			speed = &state->xSpe;
			pos = &state->xPos;
			break;
		case Y_DIM:
			speed = &state->ySpe;
			pos = &state->yPos;
			break;
	}
	

	switch(stepMode[state->id][dimension]){
		case GET_K1:
			k[0][state->id][dimension] = *speed;//*speed;
			kD[0][state->id][dimension] = acc;
			break;

		case GET_K2:
			kD[1][state->id][dimension] = acc;
			k[1][state->id][dimension] = *speed + kD[0][state->id][dimension] * dT/2;
			break;

		case GET_K3:
			kD[2][state->id][dimension] = acc;
			k[2][state->id][dimension] = *speed + kD[1][state->id][dimension] * dT/2;
			break;

		case GET_K4:
			kD[3][state->id][dimension] = acc;
			k[3][state->id][dimension] = *speed + kD[2][state->id][dimension] * dT;
			
			break;
			
		case COMPUTE:
			//printf("Id: %d\tDim: %d\tAcc: %.3f\n", state->id, dimension, acc);

			posIni[state->id][dimension] = *pos; //Update initial states
			speIni[state->id][dimension] = *speed;

			*pos = posIni[state->id][dimension] + (dT/6) * (k[0][state->id][dimension] + 2*k[1][state->id][dimension] + 2*k[2][state->id][dimension] + k[3][state->id][dimension]); //Perform this computation with the original value (since *pos will have already been increased)
			*speed = speIni[state->id][dimension] + (dT/6) * (kD[0][state->id][dimension] + 2*kD[1][state->id][dimension] + 2*kD[2][state->id][dimension] + kD[3][state->id][dimension]);
			
			break;
	}
	
	posAcc[state->id][dimension] = posIni[state->id][dimension] + (dT/6) * (k[0][state->id][dimension] + 2*k[1][state->id][dimension] + 2*k[2][state->id][dimension] + k[3][state->id][dimension]);	
	speAcc[state->id][dimension] = speIni[state->id][dimension] + (dT/6) * (kD[0][state->id][dimension] + 2*kD[1][state->id][dimension] + 2*kD[2][state->id][dimension] + kD[3][state->id][dimension]);

	if(stepMode[state->id][dimension] != COMPUTE){
		*pos = posAcc[state->id][dimension];
		*speed = speAcc[state->id][dimension];
	}
		

}

/*
	@brief	Steps the time, solves the differential equations to update the state's position and velocities
*/
void stepSystemTime(RigidState *state[], double forceMat[][MAX_OBJS][DIMENSIONS], double dT, int objCount, double *simTime){

	uint32_t i, j;

	for( i = 0; i < objCount; i++){
		if(! state[i]->fixed){
			for( j = 0; j < DIMENSIONS; j++)
			stepSingleTime(state[i], forceMat, dT, objCount, j);
		}
		
	}

	*simTime += dT;
	
	
}



