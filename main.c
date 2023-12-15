#include "main.h"

BarPlot genPlot;
void killProgram(SDL_Window *win, SDL_Renderer *ren);

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
	
	unsigned const int objCount = 3;
	//Force matrix is the matrix of the forces (in the example filled with 3 objects)
	// .----------.----------.-----------.
	// | F 0      | F 1 on 0 | F 2 on 0  |
	// :----------+----------+-----------:
	// | F 0 on 1 | F 1      | F 2 on 1  |
	// :----------+----------+-----------:
	// | F 0 on 2 | F 1 on 2 | F 2       |
	// '----------'----------'-----------'
	
	float forceMatrix[MAX_OBJS][MAX_OBJS];	
	initForceMat(forceMatrix, objCount);
	
	SolidRect objects[MAX_OBJS];	

	//ID 0 (window boundaries)
	objects[0].mass = 100;
	objects[0].dispRect.x = 0;
	objects[0].dispRect.y = 0;
	objects[0].dispRect.w = SCREEN_WIDTH;
	objects[0].dispRect.h = SCREEN_HEIGHT;
	objects[0].id = 0;

	//ID 1
	forceMatrix[1][1] = 9.81; // Id 1 --> (1,1) in matrix is its own force
	objects[1].mass = 1;
	objects[1].dispRect.h = 100;
	objects[1].dispRect.x = SCREEN_WIDTH/2-50;
	objects[1].yPos = 300;
	objects[1].dispR = 0xFF;
	objects[1].dispG = 0;
	objects[1].dispB = 0;	
	objects[1].id = 1;

	//ID 2
	forceMatrix[2][2] = 9.81;
	objects[2].mass = 1;
	objects[2].ySpeed = 0;
	objects[2].dispRect.w = 100;
	objects[2].dispRect.h = 100;
	objects[2].dispRect.x = SCREEN_WIDTH/2-150;
	objects[2].yPos = 50;
	objects[2].dispR = 0x00;
	objects[2].dispG = 0xFF;
	objects[2].dispB = 0x00;
	objects[2].id = 2;	

	genPlot.max = 20;
	genPlot.min = -20;
	genPlot.pointCount = 10000;
	genPlot.updateRate = 1;
	genPlot.callCount = 0;
	genPlot.dispRect.x = 0;
	genPlot.dispRect.y = 0;
	genPlot.dispRect.h = SCREEN_HEIGHT/2;
	genPlot.dispRect.w = SCREEN_WIDTH;
	genPlot.pointSet = (SDL_Point*) malloc(10000 * sizeof(SDL_Point));
	setRenderer(rend);

	unsigned long timeStep_mS = 0;
	unsigned long lastTime = 0;

	while(!quit){
		while(SDL_PollEvent(&e)){
			if(e.type == SDL_QUIT)
				quit = 1;
			else if(e.type == SDL_KEYDOWN){
				switch(e.key.keysym.sym){
					case SDLK_ESCAPE:
						objects[1].yPos = 0;
						objects[1].ySpeed = 0;
//						objects[2].yPos = 300;
//						objects[2].ySpeed = 0;
						break;
					case SDLK_e:
						forceMatrix[1][1]= -5;
						break;
					case SDLK_d:
						forceMatrix[1][1] = 5;
						break;
					case SDLK_w:
						forceMatrix[2][2] = -3;
						break;
					case SDLK_s:
						forceMatrix[2][2] = 3;
						break;
				}
			}
		}

				
		SDL_SetRenderDrawColor(rend, 0xFF, 0xFF, 0xFF, 0xFF);
		SDL_RenderClear(rend);
		
		timeStep_mS = SDL_GetTicks64() - lastTime;	
		SDL_Delay(1);
//		printf("%.2f FPS\n", 1.0/(timeStep_mS*0.001));
	
		stepPhys(objects, forceMatrix, objCount, timeStep_mS*0.01, 1);	
		drawSolidRect(objects[1]);
		
		stepPhys(objects, forceMatrix, objCount, timeStep_mS*0.01, 2);	
		drawSolidRect(objects[2]);
		
		lastTime = SDL_GetTicks();

		printForce(forceMatrix, objCount);

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
