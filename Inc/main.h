#define SDL_MAIN_HANDLED

#include <stdio.h> 
#include <stdlib.h>
#include <stdbool.h> 
#include <math.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include "plot.h"

#define SCREEN_WIDTH 1500
#define SCREEN_HEIGHT 1000
#define MAX_OBJS 5
#define DIMENSIONS 2
#define PLOT_COUNT 2

extern double simulationTime;

typedef struct {
	float mass;
	float ySpeed;
	float yExtForce;
	float yPos;
	SDL_Rect dispRect;
	unsigned char dispR;
	unsigned char dispG;
	unsigned char dispB;
	unsigned int id;
} SolidRect;

enum {
	X_DIM,
	Y_DIM
};
enum {
	EXTERNAL_FORCE,
	CONSTRAINT_FORCE
};
enum {
	NO_ERROR,
	ERROR
};

/*
typedef struct {
	float upLine; //Top line of the object
	float btLine; //Bottom line of the object
	float lfLine; //
	float rhLine;
}
*/

void printForce(float forceMat[][MAX_OBJS], unsigned int objCount);
void initForceMat(double forceMat[][MAX_OBJS][DIMENSIONS], unsigned int objCount);
double getTotForce(double forceMat[][MAX_OBJS][DIMENSIONS], unsigned int id, int objCount, int dimId);
void setRenderer(SDL_Renderer *ren);
void stepPhys(SolidRect rectSet[], float forceMatrix[][MAX_OBJS], int objCount, float dT_s, int id);
void drawSolidRect(SolidRect solidRect);
void forceCorrector(SolidRect *rectSet, float forceMat[][MAX_OBJS], int id, int i);
