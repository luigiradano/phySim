#include "plot.h"

#define MAX_PLOT_H 1000
#define MAX_PLOT_W 1000
#define MIN_PLOT_H 50
#define MIN_PLOT_W 50

TTF_Font *Sans;
SDL_Color Black = {0,0,0};
SDL_Color PlotTextColor = {0,0,0};
SDL_Color PlotLineColor = {0,0,0};

BarPlot genPlot[MAX_PLOT_COUNT];
int plotCount = 0;
int selectPlot = -1;
/*
	@brief	Prints str on screen with the font loaded in Sans global
*/
void printOnScreen(char *str, int x, int y, SDL_Renderer *rend){

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

/*
	@brief	Initialize a plot structure with default values, asks for the important ones
	
	@params	plot:	Plot object
			range:	Range in absolute value, a range of 200 will make a plot from -100 to 100
			x, y, w, h:	Specifiers for the displayed rectangle
			title:	Plot title (<20 char)
			ren:	SDL_Renderer object to render the graph on
*/
void initPlot(BarPlot *plot, float range, int x, int y, int w, int h, char *title, SDL_Renderer *ren){
	//Internal Math set up
	plot->max = range/2;
	plot->min = range/-2;
	plot->pointCount = 1000;
	plot->updateRate = 1;
	plot->callCount = 0;
	//Display rect set up
	plot->dispRect.x = x;
	plot->dispRect.y = y;
	plot->dispRect.w = w;
	plot->dispRect.h = h;
	//Range Set up
	plot->vertScaleRange = ((plot->dispRect.h) / (plot->max - plot->min));	
	plot->xAxisY = (plot->dispRect.y + plot->dispRect.h/2);
	//Color Set Up
	plot->lineColor.r = 0;
	plot->lineColor.g = 0;
	plot->lineColor.b = 0;
	plot->textColor.r = 0;
	plot->textColor.g = 0;
	plot->textColor.b = 0;
	plot->bgColor.r   = 0xFF;
	plot->bgColor.g   = 0xFF;
	plot->bgColor.b   = 0xFF;
	//Other Internal stuff needed
	plot->pointSet = (SDL_Point*) malloc(plot->pointCount * sizeof(SDL_Point));
	plot->rend = ren;
	strcpy(plot->title, title);
	if(plotCount == 0){
		TTF_Init();
		Sans = TTF_OpenFont("OpenSans_Condensed-Light.ttf", PLOT_TEXT_SIZE); 
	}
	
	plotCount ++;
}

/*
	@brief	Draws the plot provided in the arguments, takes currVal as the latest value received and processes it
*/
void drawPlot(BarPlot *plot, float currVal){

	plot->callCount ++;
	
	//If the function has been called less then the establised times don't change last value
	if(plot->callCount % plot->updateRate == 0)
		plot->lastVal = currVal;
	
	int horIndex = plot->callCount / plot->updateRate;
	float scaledVal = plot->vertScaleRange * plot->lastVal;
		
	int pointY = (plot->dispRect.y + plot->dispRect.h/2) - scaledVal;
	int pointX = plot->dispRect.x + (horIndex * (float)((float)plot->dispRect.w / plot->pointCount));
	
	if(pointY < plot->dispRect.y) 
		pointY = plot->dispRect.y;
	if( pointY > (plot->dispRect.y+plot->dispRect.h))
		pointY = plot->dispRect.y+plot->dispRect.h;	
	
	if(horIndex > plot->pointCount)
		plot->callCount = 0;

	plot->pointSet[horIndex].x = pointX;
	plot->pointSet[horIndex].y = pointY;

	//Print BG
	SDL_SetRenderDrawColor(plot->rend, plot->bgColor.r, plot->bgColor.g, plot->bgColor.b, 0xFF);
	SDL_RenderFillRect(plot->rend, &plot->dispRect);
	//Print texts	
	SDL_SetRenderDrawColor(plot->rend, plot->textColor.r, plot->textColor.g, plot->textColor.b, 0xFF);

	char buffer[50];
	sprintf(buffer, "%s= %.2f", plot->title, currVal);
	printOnScreen(buffer, plot->dispRect.x+plot->dispRect.w/3, plot->dispRect.y, plot->rend);

	sprintf(buffer, "%.2f", plot->max);
	printOnScreen(buffer, plot->dispRect.x, plot->dispRect.y-6, plot->rend);

	sprintf(buffer, "%.2f", plot->min);
	printOnScreen(buffer, plot->dispRect.x, plot->dispRect.y+plot->dispRect.h-30, plot->rend);

	//Print lines
	SDL_SetRenderDrawColor(plot->rend, plot->lineColor.r, plot->lineColor.g, plot->lineColor.b, 0xFF);
	SDL_RenderDrawPoints(plot->rend, plot->pointSet, plot->pointCount);
	SDL_RenderDrawLine(plot->rend, pointX, plot->dispRect.y, pointX,  plot->dispRect.y+plot->dispRect.h);
	SDL_RenderDrawLine(plot->rend, plot->dispRect.x, plot->xAxisY, plot->dispRect.x+plot->dispRect.w,  plot->xAxisY);

	SDL_RenderDrawRect(plot->rend, &plot->dispRect);
}

/*
	@brief	Displays a generic simple selection menu for the plots
	@retval	False until a selection has been made, true when selection done
	@params		 ren: SDL renderer object
			mousePos: Current mouse click position (returned from event)
*/
bool drawSelectMenu(SDL_Renderer *ren, SDL_Point mousePos){
	SDL_Rect selectionMenu;
	selectionMenu.x = 0;
	selectionMenu.y = 0;
	selectionMenu.w = 400;
	selectionMenu.h = 30;
	
	SDL_SetRenderDrawColor(ren, 0xFF, 0xFF, 0xFF, 0xFF);
	SDL_RenderFillRect(ren, &selectionMenu);
	
	SDL_SetRenderDrawColor(ren, 0x0, 0x0, 0x0, 0x0);
	SDL_RenderDrawRect(ren, &selectionMenu);
	printOnScreen("Select a plot by clicking it", 150, 0, ren);
	
	int i = 0;
	bool selectFlag = 0;
	while(i < plotCount && !selectFlag){
		if( SDL_PointInRect(&mousePos, &genPlot[i].dispRect)){
			selectFlag = 1;
			selectPlot = i;
		}
		i++;
	}	
	if(!selectFlag)
		selectPlot = -1;
	return selectFlag;
}

void drawEditMenu(SDL_Renderer *ren, int winH, int winW, unsigned int lastKey){
	
	if(selectPlot < 0)
		return;
	
	SDL_Rect editMenu;
	editMenu.w = 200;
	editMenu.x = (winW-editMenu.w)/2;
	editMenu.h = 200;
	editMenu.y = (winH-editMenu.h)/2;
/*	
	SDL_SetRenderDrawColor(ren, 0xFF, 0xFF, 0xFF, 0xFF);
	SDL_RenderFillRect(ren, &editMenu);
	
	SDL_SetRenderDrawColor(ren, 0x0, 0x0, 0x0, 0x0);
	SDL_RenderDrawRect(ren, &editMenu);
	printOnScreen("Arrows to move plot", editMenu.x, editMenu.y, ren);
	printOnScreen("Page Up/Down to widen", editMenu.x, editMenu.y+50, ren);
	printOnScreen("Home/End to change height", editMenu.x, editMenu.y+100, ren);
	printOnScreen("+/- to zoom", editMenu.x, editMenu.y+150, ren);

*/		
	switch(lastKey){
		case SDLK_KP_8:
		case SDLK_UP:
			if(genPlot[selectPlot].dispRect.y - 50 >= 0)
				genPlot[selectPlot].dispRect.y -= 50;
			break;
		case SDLK_KP_2:
		case SDLK_DOWN:
			if(genPlot[selectPlot].dispRect.y + genPlot[selectPlot].dispRect.h + 50 <= winH)
				genPlot[selectPlot].dispRect.y += 50;
			break;
		case SDLK_KP_4:
		case SDLK_LEFT:
			if(genPlot[selectPlot].dispRect.x - 50 >= 0)
				genPlot[selectPlot].dispRect.x -= 50;
			break;
		case SDLK_KP_6:
		case SDLK_RIGHT:
			if(genPlot[selectPlot].dispRect.x + genPlot[selectPlot].dispRect.w + 50 <= winW) 
				genPlot[selectPlot].dispRect.x += 50;
			break;
		case SDLK_KP_9:
		case SDLK_PAGEUP:
			if(genPlot[selectPlot].dispRect.w + 50 < MAX_PLOT_W)	
				genPlot[selectPlot].dispRect.w += 50;
			break;
		case SDLK_KP_3:
		case SDLK_PAGEDOWN:
			if(genPlot[selectPlot].dispRect.w - 50 > MIN_PLOT_W)		
				genPlot[selectPlot].dispRect.w -= 50;
			break;
		case SDLK_KP_7:
		case SDLK_HOME:
			if(genPlot[selectPlot].dispRect.h + 50 < MAX_PLOT_H)
				genPlot[selectPlot].dispRect.h += 50;
			break;
		case SDLK_KP_1:
		case SDLK_END:
			if(genPlot[selectPlot].dispRect.h - 50 > MIN_PLOT_H)
				genPlot[selectPlot].dispRect.h -= 50;
			break;
		case SDLK_KP_PLUS:
		case SDLK_PLUS:
			if(genPlot[selectPlot].max - 10 > 0){
				genPlot[selectPlot].max -= 10;
				genPlot[selectPlot].min += 10;
			}
			break;
		case SDLK_KP_MINUS:
		case SDLK_MINUS:
			genPlot[selectPlot].max += 10;
			genPlot[selectPlot].min -= 10;
			break;
		case SDLK_PERIOD:
			genPlot[selectPlot].updateRate ++;
			break;	
		case SDLK_COMMA:
			if(genPlot[selectPlot].updateRate - 1 > 0)
				genPlot[selectPlot].updateRate --;
			break;	
	}	
	
		
	genPlot[selectPlot].vertScaleRange = ((genPlot[selectPlot].dispRect.h) / (genPlot[selectPlot].max - genPlot[selectPlot].min));	
	genPlot[selectPlot].xAxisY = (genPlot[selectPlot].dispRect.y + genPlot[selectPlot].dispRect.h/2);
}
