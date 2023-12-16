#include "main.h"
#include <stdlib.h>

#define COLLISION_IGNORE_COUNT 50
#define COLL_THRESH 1
#define LOW_SPEED_THRESH 1
#define HIGH_SPEEED_THRESH 400
#define ELASTIC_LOSS 1
//#define PRINT_COLLISIONS 
//#define PRINT_SPEEDS

SDL_Renderer *rend;
TTF_Font *Sans;
SDL_Color Black = {0,0,0};

short collMat[MAX_OBJS][MAX_OBJS];

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
	if(id == 0)
		return 0;
	for(i=0; i<objCount; i++){
		totForce += forceMat[i][id];
	}
	return totForce;
}

float getRelForce(float forceMat[][MAX_OBJS], int row, int excInd, int objCount){
	return getTotForce(forceMat, row, objCount) - forceMat[row][excInd];
}

bool getForces(SolidRect *solidRect, float distance, float forceMat[][MAX_OBJS], int currIndex, int collIndex, int objCount){
	float radius = solidRect->dispRect.h/5;
	bool collFlag = false;
	//Contact interaction
	if(distance < radius && distance > 0){
		float relF_12 = getRelForce(forceMat, currIndex, collIndex, objCount);
		float relF_21 = getRelForce(forceMat, collIndex, currIndex, objCount);
		forceMat[currIndex][collIndex] =  1 * (relF_12 + relF_21);
		forceMat[collIndex][currIndex] = -1 * forceMat[currIndex][collIndex];
		collFlag = true;
	}
	else{

		forceMat[currIndex][collIndex] = 0;
		forceMat[collIndex][currIndex] = 0;
	}
	return collFlag;
}


enum{
	ABSENT,
	IMPULSE,
	STATIC
} CollisionTypes;
//Collision Types negative = timeoutCount
short checkCollision(SolidRect rectSet[], int id, int boundIndex, float forceMat[][MAX_OBJS], int objCount){
	//First check if the collision has already been recorded
	short retVal;

	//Return early since we don't want collision logic
	if(collMat[id][boundIndex] < 0){	
		collMat[id][boundIndex] ++;
		return ABSENT;
	}
			
	SolidRect *solidRect = &rectSet[id];
	SolidRect *boundRect = &rectSet[boundIndex];
		
	bool top_top = abs(solidRect->yPos - boundRect->yPos) < COLL_THRESH;
	bool btm_btm = abs((solidRect->yPos + solidRect->dispRect.h) - (boundRect->yPos + boundRect->dispRect.h)) < COLL_THRESH;
	bool btm_top = abs((solidRect->yPos + solidRect->dispRect.h) - boundRect->yPos) < COLL_THRESH;
	bool top_btm = abs(solidRect->yPos - (boundRect->yPos + boundRect->dispRect.h)) < COLL_THRESH;
	bool isCollision = top_top || top_btm || btm_top || btm_btm;
#ifdef PRINT_COLLISIONS
	printf("%d\t%d\t%d\t%d\t", top_top, top_btm, btm_top, btm_btm);
#endif

	if(collMat[id][boundIndex] == 2 ){
		//Check if collision persists (forces point each other)
		float totForce = getTotForce(forceMat, id, objCount);
		//If force is downwards and collision is from top, then we go down, no collision
		bool goDwn = totForce > 0 && (top_top || top_btm);
		//If force is upwards and collision is from bottom, then we go up, no collision
		bool goUp = totForce < 0 && (btm_btm || btm_top);
		if( goUp || goDwn || !isCollision){
			collMat[id][boundIndex] = -1 * COLLISION_IGNORE_COUNT;
			collMat[boundIndex][id] = collMat[id][boundIndex];
			retVal = ABSENT;
		}
		else {
			retVal =  STATIC;
		}
		
	}
	else if(collMat[id][boundIndex] == 1 ){
		retVal = STATIC;
	}
	else{
		//Check if collidion present
		if(isCollision){
			//We have a collision it seems
			collMat[id][boundIndex] = -1 * COLLISION_IGNORE_COUNT;
			collMat[boundIndex][id] = collMat[id][boundIndex];
			retVal = IMPULSE;
		}
		else{	
			retVal = ABSENT;
		}
	}
	
	if(retVal != ABSENT)
		collMat[id][boundIndex] = retVal;
	
	return retVal;
}

void elasticImpulse(SolidRect rectSet[], int id, int collIndex){
	float totMass = rectSet[id].mass + rectSet[collIndex].mass;
	float ySpeedInit = rectSet[id].ySpeed;

	rectSet[id].ySpeed = ((rectSet[id].mass - rectSet[collIndex].mass) / totMass) * rectSet[id].ySpeed  + (2 * rectSet[collIndex].mass / totMass) * rectSet[collIndex].ySpeed;
	rectSet[id].ySpeed *= ELASTIC_LOSS;

	rectSet[collIndex].ySpeed = ((2 * rectSet[id].mass) / totMass) * ySpeedInit + ((rectSet[collIndex].mass - rectSet[id].mass) / totMass) * rectSet[collIndex].ySpeed;
	rectSet[collIndex].ySpeed *= ELASTIC_LOSS;

}

void stepPhys(SolidRect rectSet[], float forceMat[][MAX_OBJS], int objCount, float dT_s, int id){
	float totForce=0, totAcc=0, curSpe=0, deltaY=0;
	SolidRect *solidRect = &rectSet[id];

	int i = objCount - 1;	
	//Collision logic
	while( i >= 0 ){
		//Do not check for collisions with self (always true)
		if( i != solidRect->id ){
			short collisionType = checkCollision(rectSet, id, i, forceMat, objCount);
#ifdef PRINT_COLLISIONS
			printf("- %d with %d\tColl N°: %d\n", id, i, collisionType);
#endif	
			switch(collisionType){
				case IMPULSE:
					//Elastic impact compute
					elasticImpulse(rectSet, id, i);
					//if(rectSet[id].ySpeed < LOW_SPEED_THRESH)
					//	rectSet[id].ySpeed = 0;
					break;
				case STATIC:
					totForce = getTotForce(forceMat, id, objCount);
					if(i != 0 ){
						float relF_12 = totForce - forceMat[id][i];
						float relF_21 = 0;
						relF_21 = getTotForce(forceMat, i, objCount) - forceMat[i][id];
											
						forceMat[id][i] = -1 * (relF_12 + relF_21);
						forceMat[i][id] = -1 * forceMat[id][i];
					}
					else{
						
						forceMat[id][i] = totForce - forceMat[i][id];
						forceMat[i][id] = -1 * forceMat[id][i];	
					}
					rectSet[id].ySpeed = 0;
					rectSet[i].ySpeed = 0;	
					break;
				case ABSENT:
					
					forceMat[i][id] = 0;
					forceMat[id][i] = 0;
					break;
			}
			if(solidRect->id == 1)
				drawPlot(&genPlot[2], collisionType, rend);
		}
		
			
		i--;
	}

	totForce = getTotForce(forceMat, id, objCount);
	totAcc = totForce/solidRect->mass;
	curSpe = rectSet[id].ySpeed + totAcc * dT_s;
	deltaY = (curSpe * dT_s) + 0.5*(totAcc*dT_s*dT_s);
	
	//Do not update position of bg window
	if(id == 0)
		return; 
		
	rectSet[id].yPos += deltaY;
	rectSet[id].dispRect.y = rectSet[id].yPos;
	rectSet[id].ySpeed = curSpe;

#ifdef PRINT_SPEEDS
    printf("%ID:%d\tSpe=%.3f\tFor.=%.3f\tAcc=%.3f\tyPs=%.3f\n", solidRect->id, curSpe, totForce, totAcc, solidRect->yPos);
#endif 
	if(solidRect->id == 1){
		drawPlot(&genPlot[0], curSpe, rend);
		drawPlot(&genPlot[1], totForce, rend);
	}
}

void drawSolidRect(SolidRect solidRect){
	SDL_SetRenderDrawColor(rend, solidRect.dispR, solidRect.dispG, solidRect.dispB, 0xFF);
	SDL_RenderFillRect(rend, &solidRect.dispRect);
}


