#include "main.h"
#include "rigidBody.h"

void killProgram(SDL_Window *win, SDL_Renderer *ren);
double forceMatrix[MAX_OBJS][MAX_OBJS][DIMENSIONS];	

unsigned const int objCount = 3;

int main(int argc, const char *argv){

	if(SDL_Init(SDL_INIT_VIDEO) < 0){
		printf("SDL Init Failed!\nERROR:\t%s\n", SDL_GetError());
		return 1;
	}

	SDL_Window *window = SDL_CreateWindow("Physics Sim V0.1",
			SDL_WINDOWPOS_UNDEFINED,
			SDL_WINDOWPOS_UNDEFINED,
			SCREEN_WIDTH, SCREEN_HEIGHT,
			SDL_WINDOW_SHOWN);
	if(!window){
		printf("SDL Window could not be created!\nERROR:\t%s\n", SDL_GetError());
		return 2;
	}

	SDL_Renderer *rend = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

	if(!rend){
		printf("SDL Renderer could not be initalized!\nERROR:\t%s\n", SDL_GetError());
		return 3;
	}

	bool quit = 0; //Variable to exit gracefully from program
	SDL_Event e;
	

	initPlot(&genPlot[0], 400.0, 150,   0, 400, 200, "Speed", rend);
	initPlot(&genPlot[1], 2000,  150, 250, 400, 200, "Force", rend);
	initPlot(&genPlot[2], 1E4,   150, 500, 400, 200, "Energy", rend);

	unsigned long timeStep_uS = 0;
	
	bool plotSelectionMenu = 0;
	bool plotEditMenu = 0;
	unsigned int lastKey;
	SDL_Point clickPos;
	clickPos.x = -1;
	clickPos.y = -1;	

	RigidBall ball;
	initRigidBall(&ball, 0.500, 5);	

	while(!quit){
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
//						objects[2].yPos = 0;
	//				objects[2].yPos = 300;
	//					objects[2].ySpeed = 0;
						break;
					case SDLK_a:
						forceMatrix[0][0][0] = 2;
						break;
					case SDLK_d:
						forceMatrix[0][0][0] = -2;

						break;
					case SDLK_w:
						forceMatrix[0][0][1] = 2;
						break;
					case SDLK_s:
						forceMatrix[0][0][1] = -2;
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
	
					
		SDL_SetRenderDrawColor(rend, 0xFF, 0xFF, 0xFF, 0xFF);
		SDL_RenderClear(rend);
		
		timeStep_uS = 1000;//SDL_GetTicks64() - lastTime;	
		SDL_Delay(1);
		odeSolve(&ball.state, forceMatrix, timeStep_uS*0.000001, objCount);
		drawRigidBall(rend, &ball, SCREEN_HEIGHT, SCREEN_WIDTH);
		printRigidBallState(&ball);
		drawPlot(&genPlot[1], ball.state.ySpe);
//		printForce(forceMatrix, objCount);	
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

