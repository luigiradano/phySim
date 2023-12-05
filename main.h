#include <stdio.h> 
#include <stdbool.h> 
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#define SCREEN_WIDTH 1000
#define SCREEN_HEIGHT 800
#define MAX_OBJS 5

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

/*
typedef struct {
	float upLine; //Top line of the object
	float btLine; //Bottom line of the object
	float lfLine; //
	float rhLine;
}
*/

void printForce(float forceMat[][MAX_OBJS], unsigned int objCount);
void initForceMat(float forceMat[][MAX_OBJS], unsigned int objCount);
void setRenderer(SDL_Renderer *ren);
void stepForces(SolidRect *solidRect, SDL_Rect *boundarSet[], float forceMatrix[][MAX_OBJS], unsigned int objCount, float dT_s);
void drawSolidRect(SolidRect *solidRect);

