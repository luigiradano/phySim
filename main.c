#include "main.h"
#include "rigidBody.h"
#include "constraintSolve.h"
#include "matrixOps.h"
#include <math.h>

#define TIMESTEP 0.00001 //In seconds


void killProgram(SDL_Window *win, SDL_Renderer *ren);
double forceMatrix[MAX_OBJS][MAX_OBJS][DIMENSIONS];	
char initSDL();
void pollSDL();
	
unsigned const int objCount = 3;
SDL_Window *window;
SDL_Renderer *rend;
bool plotSelectionMenu = 0;
bool plotEditMenu = 0;
unsigned int lastKey;
SDL_Point clickPos;
bool quit = 0; //Variable to exit gracefully from program

RigidBall ball;
//MAIN
int main(int argc, const char *argv){
	
	initSDL();
	
//	initPlot(&genPlot[0], 400.0, 150,   0, 400, 200, "Speed", rend)
//	initPlot(&genPlot[1], 2000,  150, 250, 400, 200, "Force", rend);
	initPlot(&genPlot[2], 200,   150, 500, 400, 200, "Energy", rend);
	
	genPlot[2].updateRate = 100;
	
	initRigidBall(&ball, 0.5, 1);	

	
	ball.state.yPos = 2;
	ball.state.xPos = 0;

/*
	Matrix A, B, C;
	initMatrix(&A, 2, 2);
	initMatrix(&B, 2, 2);
	initMatrix(&C, 2, 2);
		
	setElement(&B, 0, 0, 1);
	setElement(&B, 1, 1, 1);
	setElement(&B, 0, 1, 1);
	setElement(&B, 1, 0, 1);
		
	setElement(&A, 0, 0, 1);
	setElement(&A, 0, 1, 1);
	setElement(&A, 1, 0, 1);
	setElement(&A, 1, 1, 1);
		
	matrixMultiply(&A, &B, &C);

	printMatrix(&C);
	return 0;	
*/
	Constraint constraints[1];
	
	initConstraints(&constraints[0], getTraj, getJacob, getJacob2);
	solveConstraints(constraints, 1, &ball.state, forceMatrix);

	while(!quit){
			
		pollSDL();
			
		SDL_SetRenderDrawColor(rend, 0xFF, 0xFF, 0xFF, 0xFF);
		SDL_RenderClear(rend);
		
		
		odeSolve(&ball.state, forceMatrix, TIMESTEP, objCount);
		drawRigidBall(rend, &ball, SCREEN_HEIGHT, SCREEN_WIDTH);

		double speed = ball.state.xSpe*ball.state.xSpe + ball.state.ySpe*ball.state.ySpe;
		speed = sqrt(speed); 
		double energy = 0.5 * ball.state.mass * pow(speed, 2);
//		printRigidBallState(&ball);
		drawPlot(&genPlot[2], energy);
//		printForce(forceMatrix, objCount);	

		solveConstraints(constraints, 1, &ball.state, forceMatrix);







		if(plotSelectionMenu){
			plotEditMenu = drawSelectMenu(rend, clickPos);
			plotSelectionMenu = !plotEditMenu; //When switching to editMenu disable selection
		}
		if(plotEditMenu){
			drawEditMenu(rend, SCREEN_HEIGHT, SCREEN_WIDTH, lastKey);
			lastKey = 0;
		}
		SDL_RenderPresent(rend);
		

	}
	killProgram(window, rend);	

	return 0;
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
						ball.state.xPos = 0;
						ball.state.yPos = 2;
						ball.state.ySpe = 0;
						ball.state.xSpe = 0;
						//objects[2].yPos = 0;
	//				objects[2].yPos = 300;
	//					objects[2].ySpeed = 0;
						break;
					case SDLK_a:
						ball.state.xSpe = 10;
//						forceMatrix[0][0][0] = 20;
						break;
					case SDLK_d:
						ball.state.xSpe = -10;
//						forceMatrix[0][0][0] = -20;

						break;
					case SDLK_w:
						ball.state.ySpe = 10;
//						forceMatrix[0][0][1] = 20;
						break;
					case SDLK_s:
						ball.state.ySpe = -10;
//						forceMatrix[0][0][1] = -20;
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
	for(i=0; i<objCount; i++){
		totForce += forceMat[i][id][dimId];
	}
	return totForce;
}

