#include "main.h"

SDL_Color PlotColor = {0,0,0};


void drawPlot(BarPlot *plot, float currVal, SDL_Renderer *ren){

	plot->callCount ++;
	
	//If the function has been called less then the establised times don't change last value
	if(plot->callCount % plot->updateRate == 0)
		plot->lastVal = currVal;
	
	int horIndex = plot->callCount / plot->updateRate;
	
	float vertScaleRange = (plot->dispRect.h / (plot->max - plot->min));
	float scaledVal = vertScaleRange * ((plot->lastVal - plot->min) / (plot->max - plot->min));
	int pointY = (plot->dispRect.y + plot->dispRect.h) - scaledVal;
	int pointX = plot->dispRect.x + (horIndex * (float)((float)plot->dispRect.w / plot->pointCount));
	
	
	if(horIndex > plot->pointCount)
		plot->callCount = 0;

	float xAxisY = (plot->dispRect.y + plot->dispRect.h) - vertScaleRange * 0.5;
	
	SDL_SetRenderDrawColor(ren, PlotColor.r, PlotColor.g, PlotColor.b, 0xFF);

	plot->pointSet[horIndex].x = pointX;
	plot->pointSet[horIndex].y = pointY;
	
	SDL_RenderDrawPoints(ren, plot->pointSet, plot->pointCount);

	SDL_RenderDrawLine(ren, plot->dispRect.x, xAxisY, plot->dispRect.x + plot->dispRect.y,  xAxisY);
    	
}
