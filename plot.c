#include "main.h"

SDL_Color PlotColor = {0,0,0};

void initPlot(BarPlot *plot, float range, int x, int y, int w, int h, char *title){
	plot->max = range/2;
	plot->min = range/-2;
	plot->pointCount = 1000;
	plot->updateRate = 1;
	plot->callCount = 0;
	plot->dispRect.x = x;
	plot->dispRect.y = y;
	plot->dispRect.w = w;
	plot->dispRect.h = h;
	plot->pointSet = (SDL_Point*) malloc(plot->pointCount * sizeof(SDL_Point));
	strcpy(plot->title, title);
}

void drawPlot(BarPlot *plot, float currVal, SDL_Renderer *ren){

	plot->callCount ++;
	
	//If the function has been called less then the establised times don't change last value
	if(plot->callCount % plot->updateRate == 0)
		plot->lastVal = currVal;
	
	int horIndex = plot->callCount / plot->updateRate;
	
	float vertScaleRange = ((plot->dispRect.h) / (plot->max - plot->min));
	float scaledVal = vertScaleRange * (plot->lastVal );
	int pointY = (plot->dispRect.y + plot->dispRect.h/2) - scaledVal;
	int pointX = plot->dispRect.x + (horIndex * (float)((float)plot->dispRect.w / plot->pointCount));
	
	
	if(horIndex > plot->pointCount)
		plot->callCount = 0;

	float xAxisY = (plot->dispRect.y + plot->dispRect.h/2);
	
	SDL_SetRenderDrawColor(ren, PlotColor.r, PlotColor.g, PlotColor.b, 0xFF);

	plot->pointSet[horIndex].x = pointX;
	plot->pointSet[horIndex].y = pointY;
	
	char buffer[50];
	sprintf(buffer, "%s= %.2f", plot->title, currVal);
	printOnScreen(buffer, plot->dispRect.x+plot->dispRect.w/3, plot->dispRect.y);
	sprintf(buffer, "%.2f", plot->max);
	printOnScreen(buffer, plot->dispRect.x, plot->dispRect.y);
	sprintf(buffer, "%.2f", plot->min);
	printOnScreen(buffer, plot->dispRect.x, plot->dispRect.y+plot->dispRect.h-30);

	SDL_RenderDrawPoints(ren, plot->pointSet, plot->pointCount);
	SDL_RenderDrawRect(ren, &plot->dispRect);
	SDL_RenderDrawLine(ren, plot->dispRect.x, xAxisY, plot->dispRect.x + plot->dispRect.w,  xAxisY);
	SDL_RenderDrawLine(ren, pointX, plot->dispRect.y, pointX,  plot->dispRect.y+plot->dispRect.h);
    	
}
