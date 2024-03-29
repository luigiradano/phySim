#include "main.h"
#include "matrixOps.h"
#include "rigidBody.h"
#include "constraintSolve.h"
#include <sys/time.h>
#include <math.h>

//#define MOUSE_IS_GRAVITY
#define EXT_ACC_MODULUS 2
#define ACC_ID 0
#define TIMESTEP_PER_FPS 01
#define TIMESTEP 0.0001//In seconds

// #define SINGLESTATEDEBUG

void killProgram(SDL_Window *win, SDL_Renderer *ren);
double forceMatrix[2][MAX_OBJS][DIMENSIONS];	
char initSDL();
void pollSDL();
uint64_t micro_time();
	
unsigned const int objCount = 2;
SDL_Window *window;
SDL_Renderer *rend;
//Stuff for pltos
bool plotSelectionMenu = 0;
bool plotEditMenu = 0;
unsigned int lastKey;
SDL_Point clickPos;

bool quit = 0; //Variable to exit gracefully from program

//Trajectory drawing
SDL_Texture *trajText;


uint64_t fpsTime, fpsStart, start, solTime, steTime;

uint32_t i = 0;
double simulationTime = 0;

//MAIN
/*
int main(int argc, const char *argv){
	
	initSDL();
	
	initPlot(&genPlot[0], 100,   0, 50, 1000, 200, "Energy", rend);
	initContraintMats(objCount);

	RigidState *states[2];
	initRigidBall(&ball[0], 0.5, 0.2);	
	initRigidBall(&ball[1], 0.5, 0.2);

	ball[0].state.xPos = 0.5;
	ball[0].state.yPos = 0;
	ball[0].state.id = 0;
	states[0] = &ball[0].state;

	ball[1].state.xPos = 1;
	ball[1].state.yPos = 0;
	ball[1].state.id = 1;
	states[1] = &ball[1].state;

	forceMatrix[EXTERNAL_FORCE][0][Y_DIM] = -1 * EXT_ACC_MODULUS * ball[0].state.mass;
	forceMatrix[EXTERNAL_FORCE][1][Y_DIM] = -1 * EXT_ACC_MODULUS * ball[1].state.mass;

	setInitials(states, objCount);

	trajText = SDL_CreateTexture(rend, SDL_PIXELFORMAT_RGB888, SDL_TEXTUREACCESS_STREAMING, SCREEN_WIDTH, SCREEN_HEIGHT);

	SDL_SetRenderDrawColor(rend, 0x00, 0x00, 0x00, 0xFF);
	SDL_RenderClear(rend);
	SDL_SetTextureBlendMode(trajText, SDL_BLENDMODE_BLEND);

	while(!quit){
		
		//system("CLS");
		fpsStart = micro_time();
		pollSDL();

		//Needs to be rendered as first thing
		drawTraj(rend, &ball[1].state, trajText);
						
		char info[110];
		sprintf(info, "Sim Time: %6.3f s  Sim.Freq: %2.0f kHz Updates: %d FPS: %3.0f  Compute: %.2f ms", simulationTime, 1E-3/((float)TIMESTEP), TIMESTEP_PER_FPS, (float)1E6/fpsTime, (float)steTime/1000.0f);		
		printOnScreen(info, 0, 0, rend);
		
		drawRigidBall(rend, &ball[0], SCREEN_HEIGHT, SCREEN_WIDTH, true);
		drawRigidBall(rend, &ball[1], SCREEN_HEIGHT, SCREEN_WIDTH, false);
		
		drawLink(rend, &ball[0], &ball[1], SCREEN_HEIGHT, SCREEN_WIDTH);

		int32_t xForceFactor = 0, yForceFactor = 0;

#ifdef MOUSE_IS_GRAVITY
		SDL_GetMouseState(&xForceFactor, &yForceFactor);

		xForceFactor -= SCREEN_WIDTH/2;
		yForceFactor -= SCREEN_HEIGHT/2;
#endif

		start = micro_time();
		for(i = 0; i < TIMESTEP_PER_FPS; i ++){

			solveConstraints( states, forceMatrix, 2);
			stepSystemTime(states, forceMatrix, TIMESTEP, objCount, &simulationTime);

			forceMatrix[EXTERNAL_FORCE][0][Y_DIM] = -1 * EXT_ACC_MODULUS * ball[0].state.mass * xForceFactor / (float) SCREEN_WIDTH/2;
			forceMatrix[EXTERNAL_FORCE][1][Y_DIM] = -1 * EXT_ACC_MODULUS * ball[1].state.mass * yForceFactor / (float) SCREEN_HEIGHT/2;

			forceMatrix[EXTERNAL_FORCE][0][X_DIM] = 0;
			forceMatrix[EXTERNAL_FORCE][1][X_DIM] = 0;
		}
	
		steTime = micro_time() - start;
		
		


		double energy = 0;
		energy += getKinEne(&ball[0].state);
		energy += getPotEne(&ball[0].state, forceMatrix);
		energy += getKinEne(&ball[1].state);
		energy += getPotEne(&ball[1].state, forceMatrix);

		drawPlot(&genPlot[0], energy);



		if(plotSelectionMenu){
			plotEditMenu = drawSelectMenu(rend, clickPos);
			plotSelectionMenu = !plotEditMenu; //When switching to editMenu disable selection
		}
		if(plotEditMenu){
			drawEditMenu(rend, SCREEN_HEIGHT, SCREEN_WIDTH, lastKey);
			lastKey = 0;
		}
		SDL_RenderPresent(rend);

		fpsTime = micro_time() - fpsStart;

	}
	killProgram(window, rend);	

	return 0;
}
*/

int main(int argc, const char *argv){
	
	initSDL();
    

	RigidBeam test;
	//Initalize and register the hinge constraint
	Constraint fixOrigin;
	RigidState originSetpoint;

	RigidState stato;

	stato.xPos = 1;
	stato.yPos = 0;
	stato.xSpe = 0;
	stato.ySpe = 0;
	stato.mass = 1;

	
#ifdef SINGLESTATEDEBUG
	registerState(&stato);
	registerState(&originSetpoint);

	originSetpoint.xPos = stato.xPos+0.5;
	originSetpoint.xSpe = stato.xSpe;
	originSetpoint.yPos = stato.yPos;
	originSetpoint.ySpe = stato.ySpe;
	originSetpoint.mass = 1;
	originSetpoint.fixed = true;

	forceMatrix[EXTERNAL_FORCE][stato.id][Y_DIM] = -1 * EXT_ACC_MODULUS;
	forceMatrix[EXTERNAL_FORCE][stato.id][X_DIM] = 0;

	initHingeConstraint(&fixOrigin, stato.id, originSetpoint.id);

#else
	registerState(&originSetpoint);

	//Initalize the beam linkage
	initRigidBeam(&test, 0.5, 2, 2);
	initHingeConstraint(&fixOrigin, test.stateA.id, originSetpoint.id);

	originSetpoint.xPos = test.stateA.xPos;
	originSetpoint.xSpe = test.stateA.xSpe;
	originSetpoint.yPos = test.stateA.yPos;
	originSetpoint.ySpe = test.stateA.xSpe;
	originSetpoint.mass = 1;

	forceMatrix[EXTERNAL_FORCE][test.stateA.id][Y_DIM] = -1 * EXT_ACC_MODULUS;
	forceMatrix[EXTERNAL_FORCE][test.stateA.id][X_DIM] = 0;
	
	forceMatrix[EXTERNAL_FORCE][test.stateB.id][Y_DIM] = 1 * EXT_ACC_MODULUS;
	forceMatrix[EXTERNAL_FORCE][test.stateB.id][X_DIM] = 0;
#endif


	//Fix the position of one end of the beam
	registerConstraint(&fixOrigin);

	setInitials(globState, stateIndex); //Get inital states

	trajText = SDL_CreateTexture(rend, SDL_PIXELFORMAT_RGB888, SDL_TEXTUREACCESS_STREAMING, SCREEN_WIDTH, SCREEN_HEIGHT);

	SDL_SetRenderDrawColor(rend, 0x00, 0x00, 0x00, 0xFF);
	SDL_RenderClear(rend);
	SDL_SetTextureBlendMode(trajText, SDL_BLENDMODE_BLEND);

	initContraintMats(constIndex);

	while(!quit){
		
		fpsStart = micro_time();
		pollSDL();

#ifndef SINGLESTATEDEBUG
		drawTraj(rend, &test.stateA, trajText);
		drawLink(rend, &test.stateA,  &test.stateB, SCREEN_HEIGHT, SCREEN_WIDTH);
#else
		drawTraj(rend, &stato, trajText);
#endif
				
		// char info[110];
		// sprintf(info, "Sim Time: %6.3f s  Sim.Freq: %2.0f kHz Updates: %d FPS: %3.0f  Compute: %.2f ms", simulationTime, 1E-3/((float)TIMESTEP), TIMESTEP_PER_FPS, (float)1E6/fpsTime, (float)steTime/1000.0f);		
		// printOnScreen(info, 0, 0, rend);

		
		int32_t xForceFactor = 0, yForceFactor = 0;

#ifdef MOUSE_IS_GRAVITY
		SDL_GetMouseState(&xForceFactor, &yForceFactor);

		xForceFactor -= SCREEN_WIDTH/2;
		yForceFactor -= SCREEN_HEIGHT/2;
#endif

		start = micro_time();
		for(i = 0; i < TIMESTEP_PER_FPS; i ++){

			if (solveConstraintSystem(globConst, globState, forceMatrix, constIndex) == ERROR){
				printf("Error in constraint solution!\n");
			}
			
			stepSystemTime(globState, forceMatrix, TIMESTEP, stateIndex, &simulationTime);

#ifdef SINGLESTATEDEBUG
			forceMatrix[EXTERNAL_FORCE][stato.id][Y_DIM] = -1 * EXT_ACC_MODULUS;
			forceMatrix[EXTERNAL_FORCE][stato.id][X_DIM] = 0;
#else
			forceMatrix[EXTERNAL_FORCE][test.stateA.id][Y_DIM] = -1 * EXT_ACC_MODULUS;
			forceMatrix[EXTERNAL_FORCE][test.stateA.id][X_DIM] = 0;
			forceMatrix[EXTERNAL_FORCE][test.stateB.id][Y_DIM] = 1 * EXT_ACC_MODULUS;
			forceMatrix[EXTERNAL_FORCE][test.stateB.id][X_DIM] = 0;
#endif
		}
	
		steTime = micro_time() - start;

		printf("Origin:\t\t");
		printState(globState[0], forceMatrix);
		printf("State A:\t");
		printState(globState[1], forceMatrix);
#ifndef SINGLESTATEDEBUG
		printf("State B:\t");	
		printState(globState[2], forceMatrix);
		printDistance(&test.stateA, &test.stateB);
#endif

		SDL_RenderPresent(rend);

		fpsTime = micro_time() - fpsStart;

	}
	killProgram(window, rend);	

	return 0;
}

uint64_t micro_time() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * (uint64_t)1000000 + tv.tv_usec;
}

void killProgram(SDL_Window *win, SDL_Renderer *ren){
	if(!win)
		SDL_DestroyWindow(win);
	if(!ren)
		SDL_DestroyRenderer(ren);
	SDL_Quit();
}
void pollSDL(){
	
		SDL_Event e;
		while(SDL_PollEvent(&e)){
			
			if(e.type == SDL_QUIT)
				quit = 1;
			else if(e.type == SDL_MOUSEBUTTONDOWN){
				if(e.button.button == SDL_BUTTON_LEFT){
					clickPos.x = e.motion.x;
					clickPos.y = e.motion.y;			
				}
			}
			else if(e.type == SDL_KEYDOWN){
				lastKey = e.key.keysym.sym;
				switch(e.key.keysym.sym){
					case SDLK_ESCAPE:

						forceMatrix[EXTERNAL_FORCE][0][X_DIM] = 0;
						forceMatrix[EXTERNAL_FORCE][0][Y_DIM] = 0;

						forceMatrix[EXTERNAL_FORCE][1][X_DIM] = 0;
						forceMatrix[EXTERNAL_FORCE][1][Y_DIM] = 0;

						break;

					case SDLK_a:
//						ball.state.xSpe = 10;
						forceMatrix[EXTERNAL_FORCE][ACC_ID][X_DIM] = EXT_ACC_MODULUS ;
						forceMatrix[EXTERNAL_FORCE][1][X_DIM] = EXT_ACC_MODULUS ;

						break;
					case SDLK_d:
//						ball.state.xSpe = -10;
						forceMatrix[EXTERNAL_FORCE][ACC_ID][X_DIM] =  -1 * EXT_ACC_MODULUS ;
						forceMatrix[EXTERNAL_FORCE][1][X_DIM] =  -1 * EXT_ACC_MODULUS ;
						break;
					case SDLK_w:
//						ball.state.ySpe = 10;
						forceMatrix[EXTERNAL_FORCE][ACC_ID][Y_DIM] = EXT_ACC_MODULUS;
						forceMatrix[EXTERNAL_FORCE][1][Y_DIM] = EXT_ACC_MODULUS ;

						break;
						
					case SDLK_s:
//						ball.state.ySpe = -10;
						forceMatrix[EXTERNAL_FORCE][ACC_ID][Y_DIM] = -1 * EXT_ACC_MODULUS ;
						forceMatrix[EXTERNAL_FORCE][1][Y_DIM] = -1 * EXT_ACC_MODULUS ;

						break;
					case SDLK_p:
						plotSelectionMenu = 1;
						break;
					case SDLK_q:
						plotSelectionMenu = 0;
						plotEditMenu = 0;
						break;
				}
			}
		}
}
char initSDL(){

	if(SDL_Init(SDL_INIT_VIDEO) < 0){
		printf("SDL Init Failed!\nERROR:\t%s\n", SDL_GetError());
		return 1;
	}

	window = SDL_CreateWindow("Physics Sim V0.1",
			SDL_WINDOWPOS_UNDEFINED,
			SDL_WINDOWPOS_UNDEFINED,
			SCREEN_WIDTH, SCREEN_HEIGHT,
			SDL_WINDOW_SHOWN);
	if(!window){
		printf("SDL Window could not be created!\nERROR:\t%s\n", SDL_GetError());
		return 2;
	}

	rend = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

	if(!rend){
		printf("SDL Renderer could not be initalized!\nERROR:\t%s\n", SDL_GetError());
		return 3;
	}
}
//Set up forceMat to 0s
void initForceMat(double forceMat[][MAX_OBJS][DIMENSIONS], unsigned int objCount){
	int i,j,k;
	for(i=0; i<objCount; i++){
		for(j=0; j<objCount; j++){
			for(k=0; k<objCount; k++){
				forceMat[i][j][k] = 0;
			}
		}
	}
}

double getTotForce(double forceMat[][MAX_OBJS][DIMENSIONS], unsigned int id ,int objCount, int  dimId){
	double totForce = 0;
	int i;
	for(i=0; i<2; i++){
		totForce += forceMat[i][id][dimId];
	}
	return totForce;
}

void printForce(double forceMat[][MAX_OBJS][DIMENSIONS], unsigned int objCount){
	uint32_t i, j, k;
	for( i = 0; i < CONSTRAINT_FORCE; i ++){
		if(i == EXTERNAL_FORCE)
			printf("\nExternal Forces, ");
		else
			printf("\nInternal Forces, ");
		
		for( j = 0; j < DIMENSIONS; j++){
			if(j == X_DIM)
				printf(" X dim:");
			else
				printf(" Y dim:");
			
			for(k = 0; k < objCount; k ++){
				printf( "\t%.3f,", forceMat[i][j][k]);
			}
		}
	}
}
