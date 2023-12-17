#include "main.h"
#include <stdlib.h>

#define COLLISION_IGNORE_COUNT 50
#define COLL_THRESH 1
#define LOW_SPEED_THRESH 1
#define HIGH_SPEED_THRESH 400
#define ELASTIC_LOSS 1.05
//#define PRINT_COLLISIONS 
//#define PRINT_SPEEDS

SDL_Renderer *rend;

short collMat[MAX_OBJS][MAX_OBJS];

//Init helpers
void setRenderer(SDL_Renderer *ren){
	TTF_Init();
	rend = ren;
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

void printForce(float forceMat[][MAX_OBJS], unsigned int objCount){
	char outString[100];	
	int i,j;
	printOnScreen("Forces matrix:", 0,0, rend);
	
	for(i=0; i<objCount; i++){
		for(j=0; j<objCount; j++){
			sprintf(outString, "%2.1f", forceMat[i][j]);
			printOnScreen(outString, (i) * 40, (j+1) * 40, rend);
		}
	}
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

void stepPhys(SolidRect rectSet[], float forceMat[][MAX_OBJS], int objCount, float dT_s, int id){
	float totForce=0, totAcc=0, curSpe=0, deltaY=0;
	
	totForce = getTotForce(forceMat, id, objCount);
	totAcc = totForce/rectSet[id].mass;
	curSpe = rectSet[id].ySpeed + totAcc * dT_s;
	deltaY = (curSpe * dT_s) + 0.5*(totAcc*dT_s*dT_s);
	
	//Do not update position of bg window
	if(id == 0)
		return; 
	
	if(curSpe > HIGH_SPEED_THRESH){
		curSpe = HIGH_SPEED_THRESH;
		printf("WARN: Max speed reached + !\n");
	}
	else if(curSpe < -1 * HIGH_SPEED_THRESH){
		curSpe = -1 * HIGH_SPEED_THRESH;
		printf("WARN: Max speed reached - !\n");
	}
			
	
	rectSet[id].yPos += deltaY*10000;
	rectSet[id].dispRect.y = rectSet[id].yPos ;
	rectSet[id].ySpeed = curSpe;

#ifdef PRINT_SPEEDS
    printf("%ID:%d\tSpe=%.3f\tFor.=%.3f\tAcc=%.3f\tyPs=%.3f\n", id, curSpe, totForce, totAcc, rectSet[id].yPos);
#endif 
	if(id == 2){
		drawPlot(&genPlot[0], curSpe);
		drawPlot(&genPlot[1], totForce);
	}
}

void drawSolidRect(SolidRect solidRect){
	SDL_SetRenderDrawColor(rend, solidRect.dispR, solidRect.dispG, solidRect.dispB, 0xFF);
	SDL_RenderFillRect(rend, &solidRect.dispRect);
}


