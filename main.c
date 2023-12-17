#include "main.h"

void killProgram(SDL_Window *win, SDL_Renderer *ren);

SolidRect objects[MAX_OBJS];	
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
	

	//ID 0 (window boundaries)
	objects[0].mass = 100;
	objects[0].dispRect.x = 0;
	objects[0].dispRect.y = 0;
	objects[0].dispRect.w = SCREEN_WIDTH;
	objects[0].dispRect.h = SCREEN_HEIGHT;
	objects[0].id = 0;

	//ID 1
	forceMatrix[1][1] = 0; // Id 1 --> (1,1) in matrix is its own force
	objects[1].mass = 0.1;
	objects[1].dispRect.h = 100;
	objects[1].dispRect.w = 100;
	objects[1].dispRect.x = SCREEN_WIDTH/2+150;
	objects[1].yPos = 300;
	objects[1].dispR = 0xFF;
	objects[1].dispG = 0;
	objects[1].dispB = 0;	
	objects[1].id = 1;

	//ID 2
	forceMatrix[2][2] = 0;
	objects[2].mass = 100;
	objects[2].ySpeed = 0;
	objects[2].dispRect.w = 100;
	objects[2].dispRect.h = 100;
	objects[2].dispRect.x = SCREEN_WIDTH/2+175;
	objects[2].yPos = 50;
	objects[2].dispR = 0x00;
	objects[2].dispG = 0xFF;
	objects[2].dispB = 0x00;
	objects[2].id = 2;	

	setRenderer(rend);
	
	initPlot(&genPlot[0], 400.0, 150,   0, 400, 200, "Speed", rend);
	initPlot(&genPlot[1], 2000,  150, 250, 400, 200, "Force", rend);
	initPlot(&genPlot[2], 6.0,   150, 500, 400, 200, "Coll.", rend);

	unsigned long timeStep_mS = 0;
	unsigned long lastTime = 0;
	bool plotSelectionMenu = 0;
	bool plotEditMenu = 0;
	unsigned long piCount = 0;
	unsigned int lastKey;
	SDL_Point clickPos;
	clickPos.x = -1;
	clickPos.y = -1;	
	bool wasSpeedPositive = 0;
	
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
						objects[2].ySpeed = 0;
	//				objects[2].yPos = 300;
	//					objects[2].ySpeed = 0;
						break;
					case SDLK_e:
						//forceMatrix[2][2]= -15;
						objects[2].ySpeed = -20;
						break;
					case SDLK_d:
						//forceMatrix[2][2] = 15;
						objects[2].ySpeed = 20;
						break;
					case SDLK_w:
						forceMatrix[1][1] = -30;
						break;
					case SDLK_s:
						forceMatrix[1][1] = 30;
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
		
		timeStep_mS = 1;//SDL_GetTicks64() - lastTime;	
		SDL_Delay(1);
	//		printf("%.2f FPS\n", 1.0/(timeStep_mS*0.001));
	
		
		stepPhys(objects, forceMatrix, objCount, timeStep_mS*0.01, 1);	
		drawSolidRect(objects[1]);
		
		if(objects[1].ySpeed > 0 && !wasSpeedPositive){
			wasSpeedPositive = true;
			piCount ++;
		}	
		else if(objects[1].ySpeed < 0 && wasSpeedPositive){
			wasSpeedPositive = false;
			piCount ++;
		}	
		
		printf("%d = PI?\n", piCount);

		stepPhys(objects, forceMatrix, objCount, timeStep_mS*0.01, 2);	
		drawSolidRect(objects[2]);

		for(int i = 0; i < objCount-1; i++){
			for(int j = 1; j < objCount; j++){
				if(i != j){
					forceCorrector(objects, forceMatrix, i, j);
				}
			}
		}
			
		lastTime = SDL_GetTicks();
		printForce(forceMatrix, objCount);
		
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
