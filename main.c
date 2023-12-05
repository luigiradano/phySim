#include "main.h"

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
	SDL_Rect *boundarySet [objCount]; //Number of objects to interact + 1 (window ID: 0)
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
	//Set window boundaries
	SDL_Rect rootWindow;
	rootWindow.y = 0;
	rootWindow.h = SCREEN_HEIGHT;
	boundarySet[0] = &rootWindow;

	SolidRect test; //ID 1
	forceMatrix[1][1] = 9.81; // Id 1 --> (1,1) in matrix is its own force
	test.mass = 1;
	test.ySpeed = 0;
	test.dispRect.w = 100;
	test.dispRect.h = 100;
	test.dispRect.x = SCREEN_WIDTH/2-50;
	test.yPos = 300;
	test.dispR = 0xFF;
	test.dispG = 0;
	test.dispB = 0;	
	test.id = 1;

	SolidRect prov; //ID 2
	forceMatrix[2][2] = 9.81;
	prov.mass = 1;
	prov.ySpeed = 0;
	prov.dispRect.w = 100;
	prov.dispRect.h = 100;
	prov.dispRect.x = SCREEN_WIDTH/2-150;
	prov.yPos = 50;
	prov.dispR = 0x00;
	prov.dispG = 0xFF;
	prov.dispB = 0x00;
	prov.id = 2;	

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
						test.yPos = 0;
						test.ySpeed = 0;
						break;
					case SDLK_e:
						forceMatrix[1][1]= -15;
						break;
					case SDLK_d:
						forceMatrix[1][1] = 15;
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
//		SDL_Delay(10);	
		printf("%.2f FPS\n", 1.0/(timeStep_mS*0.001));
	
		stepForces(&prov, boundarySet, forceMatrix, objCount, timeStep_mS*0.01);	
		drawSolidRect(&prov);
		boundarySet[2] = &prov.dispRect;
		stepForces(&test, boundarySet, forceMatrix, objCount, timeStep_mS*0.01);	
		drawSolidRect(&test);
		boundarySet[1] = &test.dispRect;
			
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
