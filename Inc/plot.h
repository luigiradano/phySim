#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define PLOT_TEXT_SIZE 24
#define MAX_PLOT_COUNT 3
#define MAX_TITLE_LEN 20
#define DISPLAY_VARS_COUNT 4

typedef struct{
	char title[MAX_TITLE_LEN];
	float max; //Max value to display
	float min; //Min value to display 
	float vertScaleRange; //Internal working, do not change
	float xAxisY; //Internal working, do not change
	unsigned int pointCount; //Max points to display
	unsigned int updateRate; //How many times doed drawPlot have to be called before updating the displayed plot?
	unsigned int callCount; //Internal handling, set to 0 on init
	float lastVal; //Internal handlin, set to 0 on init
	SDL_Rect dispRect; //Rectangle describing the plot
	SDL_Point *pointSet; //Point set to display
	SDL_Renderer *rend; //Renderer
	SDL_Color textColor;
	SDL_Color lineColor;
	SDL_Color bgColor;
	bool selected;
} BarPlot;

extern BarPlot genPlot[MAX_PLOT_COUNT];

void printOnScreen(char *str, int x, int y, SDL_Renderer *rend);
void drawPlot(BarPlot *plot, float currVal);
void initPlot(BarPlot *plot, float range, int x, int y, int w, int h, char *title, SDL_Renderer *ren);
bool drawSelectMenu(SDL_Renderer *ren, SDL_Point mousePos);
void drawEditMenu(SDL_Renderer *ren, int winH, int winW, unsigned int lastKey);
