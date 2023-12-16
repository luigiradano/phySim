#include <stdio.h> 
#include <stdbool.h> 
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#define SCREEN_WIDTH 1000
#define SCREEN_HEIGHT 800
#define MAX_OBJS 5
#define PLOT_COUNT 3

typedef struct{
	char title[20];
	float max; //Max value to display
	float min; //Min value to display 
	unsigned int pointCount; //Max points to display
	unsigned int updateRate; //How many times doed drawPlot have to be called before updating the displayed plot?
	unsigned int callCount;
	float lastVal;
	SDL_Rect dispRect;
	SDL_Point *pointSet;
} BarPlot;

extern BarPlot genPlot[PLOT_COUNT];

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
void stepPhys(SolidRect rectSet[], float forceMatrix[][MAX_OBJS], int objCount, float dT_s, int id);
void drawSolidRect(SolidRect solidRect);
void printOnScreen(char *str, int x, int y);
void drawPlot(BarPlot *plot, float currVal, SDL_Renderer *ren);
void initPlot(BarPlot *plot, float range, int x, int y, int w, int h, char *title);
