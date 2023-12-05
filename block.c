#include "main.h"
#include <stdlib.h>

#define G_CONSTANT 9.81
#define COLL_THRESH 5

SDL_Renderer *rend;
TTF_Font *Sans;
SDL_Color Black = {0,0,0};

bool collMat[MAX_OBJS][MAX_OBJS];

//Init helpers
void setRenderer(SDL_Renderer *ren){
	TTF_Init();
	rend = ren;
	Sans = TTF_OpenFont("OpenSans_Condensed-Light.ttf", 24);
}

void initForceMat(float forceMat[][MAX_OBJS], unsigned int objCount){
	int i,j;
	for(i=0; i<objCount; i++){
		for(j=0; j<objCount; j++){
			forceMat[i][j] = 0;
		}
	}
}
/*
void printForce(float forceMat[][MAX_OBJS], unsigned int objCount){
	int i,j;
	for(i=0; i<objCount; i++){
		for(j=0; j<objCount; j++){
			printf("%2.1f|\t", forceMat[i][j]);
		}
		printf("\n\n");
		
	}
}
*/

void printOnScreen(char *str, int x, int y){

	SDL_Surface *messSurf = TTF_RenderText_Solid(Sans, str, Black);

	SDL_Texture *message = SDL_CreateTextureFromSurface(rend, messSurf);
	
	SDL_Rect textRect;
	textRect.x = x;
	textRect.y = y;
	TTF_SizeText(Sans, str, &textRect.w, &textRect.h);	

	SDL_RenderCopy(rend, message, NULL, &textRect);
	SDL_FreeSurface(messSurf);
	SDL_DestroyTexture(message);
}

void printForce(float forceMat[][MAX_OBJS], unsigned int objCount){
	char outString[10];	
	int i,j;
	printOnScreen("Forces matrix:", 0,0);
	
	for(i=0; i<objCount; i++){
		for(j=0; j<objCount; j++){
			sprintf(outString, "%d", collMat[i][j]);
			printOnScreen(outString, (i) * 40, (j+1) * 40);
		}
	}
}

//DOWN IS PLUS
//Exert force of id1 on id2 blocks
void excForce(float forceMat[][MAX_OBJS], int id1, int id2){
	forceMat[id1][id2] = forceMat[id1][id1];
	forceMat[id2][id1] = forceMat[id1][id2];
}
//Remove relative forces of id1 id2 blocks
void resForce(float forceMat[][MAX_OBJS], int id1, int id2){
	forceMat[id1][id2] = 0;
	forceMat[id2][id1] = 0;
}

void stepForces(SolidRect *solidRect, SDL_Rect *boundarySet[], float forceMat[][MAX_OBJS], unsigned int objCount, float dT_s){
	//ToDo: Implement collision logic
	float totForce = 0;
	//Compute total force on block caused from other blocks
	int i;
	for(i=0; i<objCount; i++){
		totForce += forceMat[solidRect->id][i];
	}

	float totAcc = totForce*solidRect->mass;
	float curSpe = solidRect->ySpeed + (totAcc * dT_s);
	//Update position temporarely
	solidRect->yPos = solidRect->yPos + (solidRect->ySpeed * dT_s) + 0.5*(totAcc*dT_s*dT_s);
	
	i = 0;	
	bool collFlag = false;
	//Collision logic
	while( i < objCount && !collFlag){
		
		bool isColliding = (collMat[i][solidRect->id]);

		//Do not check for collisions with self (always true)
		if( i != solidRect->id && !isColliding){
			
			bool goingDown = (curSpe >= 0 );//|| totAcc > 0);
			bool goingUp =  !(curSpe >= 0 );//&& totAcc > 0);
			bool forceUp = totForce >= 0;
			
			
			//If there is a collision with the top rect and top bound, do collision only if going up
			if( abs(solidRect->yPos - boundarySet[i]->y) < COLL_THRESH){
				solidRect->yPos = boundarySet[i]->y;
				curSpe = 0;
				collFlag = true;
				printf("Collision top %d with top %d\n", solidRect->id, i);
			}
			//If there is a collision with the bottom rect and bottom bound, do collision if going down
			else if( abs((solidRect->yPos + solidRect->dispRect.h) - (boundarySet[i]->h + boundarySet[i]->y)) < COLL_THRESH ){
				solidRect->yPos = boundarySet[i]->y + boundarySet[i]->h - solidRect->dispRect.h;
				curSpe = 0;
				collFlag = true;
				printf("Collision bottom %d with bottom %d\n", solidRect->id, i);
			}
			//If there is a collision with the bottom rect and top bound, do collision if going down
			else if( abs((solidRect->yPos + solidRect->dispRect.h) - boundarySet[i]->y) < COLL_THRESH && goingDown){
				solidRect->yPos = boundarySet[i]->y - solidRect->dispRect.h;
				curSpe = 0;
				collFlag = true;
				printf("Collision bottom  %d with top %d\n", solidRect->id, i);
				
			}
			//Id there is a collision with the top rect and bottom bound, do collision if going up
			else if( abs(solidRect->yPos - (boundarySet[i]->h + boundarySet[i]->y)) < COLL_THRESH && goingUp){
				solidRect->yPos = boundarySet[i]->y + boundarySet[i]->h;
				curSpe = 0;
				collFlag = true;
				forceMat[i][solidRect->id] = forceMat[solidRect->id][solidRect->id];
				forceMat[solidRect->id][i] = forceMat[i][solidRect->id];
				printf("Collision top %d with bottom %d\n", solidRect->id, i);

			}


			if(collFlag){
				collMat[solidRect->id][i] = true;
				excForce(forceMat, solidRect->id, i);
			}
			else{
				resForce(forceMat, solidRect->id, i);
				collMat[solidRect->id][i] = false;
			}
		
		}
		
		i++;
	}	
		

/*
	if(solidRect->yPos < 0){
		solidRect->yPos = 0;
		curSpe = 0;
	}

	if( solidRect->yPos > (SCREEN_HEIGHT - solidRect->dispRect.h)){
		solidRect->yPos = SCREEN_HEIGHT - solidRect->dispRect.h;
		curSpe = 0;
	}
*/	
	solidRect->dispRect.y = (int) solidRect->yPos;
	solidRect->ySpeed = curSpe;
}

void drawSolidRect(SolidRect *solidRect){
	SDL_SetRenderDrawColor(rend, solidRect->dispR, solidRect->dispG, solidRect->dispB, 0xFF);
	SDL_RenderFillRect(rend, &solidRect->dispRect);
}


