#include "main.h"
#include <stdlib.h>

#define G_CONSTANT 9.81
#define COLL_THRESH 3
//#define PRINT_COLLISIONS 
//#define PRINT_SPEEDS

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
			sprintf(outString, "%2.1f", forceMat[i][j]);
			printOnScreen(outString, (i) * 40, (j+1) * 40);
		}
	}
}

//DOWN IS PLUS
//Exert force of id1 on id2 blocks
void excForce(float forceMat[][MAX_OBJS], float setForce, int id1, int id2){
	forceMat[id2][id1] = setForce;
	forceMat[id1][id2] = -1 * forceMat[id2][id1];
}

//Remove relative forces of id1 id2 blocks
void resForce(float forceMat[][MAX_OBJS], int id1, int id2){
	forceMat[id1][id2] = 0;
	forceMat[id2][id1] = 0;
}

float getTotForce(float forceMat[][MAX_OBJS], int id ,int objCount){
	float totForce = 0;
	//Compute total force on block caused from other blocks
	int i;
	for(i=0; i<objCount; i++){
		totForce += forceMat[i][id];
	}
	return totForce;
}

void stepForces(SolidRect *solidRect, SDL_Rect *boundarySet[], float forceMat[][MAX_OBJS], unsigned int objCount, float dT_s){
	//ToDo: Implement collision logic
	float totForce = getTotForce(forceMat, solidRect->id, objCount);
	
	
	float totAcc=0, curSpe=0, deltaY=0;
	
	int i = 0;	
	bool collFlag = false;
	bool genColl = false;
	//Collision logic
	while( i < objCount ){
		collFlag = false;
		//Do not check for collisions with self (always true)
		if( i != solidRect->id && !collMat[i][solidRect->id]){
			
			bool goingDown = (curSpe >= 0 );//|| totAcc > 0);
			bool goingUp =  !(curSpe >= 0 );//||  totAcc > 0);
			
			//If there is a collision with the top rect and top bound, do collision only if going up
			if( abs(solidRect->yPos - boundarySet[i]->y) < COLL_THRESH ){
//				solidRect->yPos = boundarySet[i]->y;
				collFlag = true;
#ifdef PRINT_COLLISIONS
				printf("Collision top %d with top %d\n", solidRect->id, i);
#endif
			}
			//If there is a collision with the bottom rect and bottom bound, do collision if going down
			else if( abs((solidRect->yPos + solidRect->dispRect.h) - (boundarySet[i]->h + boundarySet[i]->y)) < COLL_THRESH ){
//				solidRect->yPos = boundarySet[i]->y + boundarySet[i]->h - solidRect->dispRect.h;
				collFlag = true;
#ifdef PRINT_COLLISIONS
				printf("Collision bottom %d with bottom %d\n", solidRect->id, i);
#endif
			}
			//If there is a collision with the bottom rect and top bound, do collision if going down
			else if( abs((solidRect->yPos + solidRect->dispRect.h) - boundarySet[i]->y) < COLL_THRESH ){
//				solidRect->yPos = boundarySet[i]->y - solidRect->dispRect.h;
				collFlag = true;
#ifdef PRINT_COLLISIONS
				printf("Collision bottom  %d with top %d\n", solidRect->id, i);
#endif				
			}
			//Id there is a collision with the top rect and bottom bound, do collision if going up
			else if( abs(solidRect->yPos - (boundarySet[i]->h + boundarySet[i]->y)) < COLL_THRESH ){
//				solidRect->yPos = boundarySet[i]->y + boundarySet[i]->h;
				collFlag = true;
#ifdef PRINT_COLLISIONS
				printf("Collision top %d with bottom %d\n", solidRect->id, i);
#endif
			}


			if(collFlag){
				genColl = true;
				collMat[solidRect->id][i] = true;
				float relForce = totForce - forceMat[i][solidRect->id];
				bool forceAdd = ((relForce > 0) && goingDown) || ((relForce < 0) && goingUp);

				if(!forceAdd)
					relForce *= -1;

				excForce(forceMat, relForce, solidRect->id, i);
			}
			else{
				resForce(forceMat, i, solidRect->id);
				collMat[solidRect->id][i] = false;
			}
		}	
		
		i++;
	}	

	totAcc = totForce*solidRect->mass;
	deltaY = (solidRect->ySpeed * dT_s) + 0.5*(totAcc*dT_s*dT_s);

if (totForce != 0) 
	curSpe = solidRect->ySpeed + (totAcc * dT_s);
else
	curSpe = 0;

#ifdef PRINT_SPEEDS
    printf("%ID:%d\tSpe=%.3f\tFor.=%.3f\tAcc=%.3f\tyPs=%.3f\n", solidRect->id, curSpe, totForce, totAcc, solidRect->yPos);
#endif 
	
	solidRect->yPos += deltaY;
	solidRect->dispRect.y = (int) solidRect->yPos;
	solidRect->ySpeed = curSpe;
	
	if(solidRect->id == 1)
		drawPlot(&genPlot, curSpe, rend);		
}

void drawSolidRect(SolidRect *solidRect){
	SDL_SetRenderDrawColor(rend, solidRect->dispR, solidRect->dispG, solidRect->dispB, 0xFF);
	SDL_RenderFillRect(rend, &solidRect->dispRect);
}


