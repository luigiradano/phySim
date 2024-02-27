#include "main.h"
#include "rigidBody.h"

#define TRAJ_POINTS 500

SDL_PointColor trajectory[TRAJ_POINTS];
uint32_t trajIndex = 0;

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

void initRigidBeam(RigidBeam *beam, double length, double mass, double thickness){
	beam->stateA.xPos = 0.000;
	beam->stateA.xSpe = 0.000;
	beam->stateA.yPos = 0.000;
	beam->stateA.ySpe = 0.000;
	beam->stateA.theta= 0.000;
	beam->stateA.aSpe = 0.000;

	beam->stateA.mass = mass/2.0f;
	beam->stateB.mass = mass/2.0f;

	beam->length = length;
	beam->thickness = thickness;
}

void printRigidBallState(RigidBall *ball){
	printf("ID: %d\tX: %.3f\tY: %.3f\tX Speed:%.2f\tY Speed:%.2f\n", ball->state.id, ball->state.xPos, ball->state.yPos, ball->state.xSpe, ball->state.ySpe);
}

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


double getKinEne(RigidState *state){
	//E_k = 0.5 * m * v^2
	//v^2 = vx^2 + vy^2
	return 0.5 * state->mass * (pow(state->xSpe, 2) + pow(state->ySpe, 2));
}

double getPotEne(RigidState *state, double forceMat[][MAX_OBJS][DIMENSIONS]){
	//E_k = m * g * h
	return -1 * state->yPos * forceMat[EXTERNAL_FORCE][state->id][Y_DIM]; 
}

/*
	@brief Draws a link between ball and ball2
*/
void drawLink(SDL_Renderer *ren, RigidBall *ball, RigidBall *ball2, unsigned int winH, unsigned int winW){
	
	int x1 = winW/2 - ball->state.xPos * PIXELS_PER_METER + X_OFFSET;
	int y1 = winH/2 - ball->state.yPos * PIXELS_PER_METER + Y_OFFSET;
	
	int x2 = winW/2 - ball2->state.xPos * PIXELS_PER_METER + X_OFFSET;
	int y2 = winH/2 - ball2->state.yPos * PIXELS_PER_METER + Y_OFFSET;
	
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

	if(!(x1 < 0 || y1 < 0 || x1 > w || y1 > h)){

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
			trajectory[i].radius -= 0.01;
			TextureDrawCircle(trajectory[i].x, trajectory[i].y, trajectory[i].radius, pixels, rowLen, trajectory[i].color);
		}

	}

	SDL_UnlockTexture(trajHandle);
	
	SDL_RenderCopy(ren, trajHandle, NULL, NULL);

}

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

void setInitials(RigidState *states[], int objCount){
	uint32_t i, j;
	for(i = 0; i < objCount; i ++){
		
		posIni[i][X_DIM] = states[i]->xPos;
		posIni[i][Y_DIM] = states[i]->yPos;
		speIni[i][X_DIM] = states[i]->xSpe;
		speIni[i][Y_DIM] = states[i]->ySpe;
		
	}
}

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
void stepTime(RigidState *state[], double forceMat[][MAX_OBJS][DIMENSIONS], double dT, int objCount, double *simTime){

	uint32_t i, j;

	for( i = 0; i < objCount; i++){
		for( j = 0; j < DIMENSIONS; j++)
			stepSingleTime(state[i], forceMat, dT, objCount, j);
	}

	*simTime += dT;
	
	
}



