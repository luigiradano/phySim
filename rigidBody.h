#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#define PIXELS_PER_METER 400
#define X_OFFSET 0
#define Y_OFFSET 0
#define BALL_SIZE 20
#define MAX_OBJS 5
#define DIMENSIONS 2

//State object for generic rigid body
typedef struct{
	double xPos; // m
	double yPos; // m  
	double theta;// rad
	
	double xSpe; // m/s
	double ySpe; // m/s
	double aSpe; // Angular speed (rad/s)
	
	double mass; // kg	

	unsigned int id; //Used to get forces	
	
} RigidState;


//Rigid body definitions
typedef struct {
	double radius;
	RigidState state;
} RigidBall;

void initRigidBall(RigidBall *ball, double radius, double mass);

void printRigidBallState(RigidBall *ball);

void drawLink(SDL_Renderer *ren, RigidBall *ball, RigidBall *ball2,unsigned int winH, unsigned int winW);
void drawTraj(SDL_Renderer *ren, RigidBall *ball, SDL_Texture *trajHandle);
void drawRigidBall(SDL_Renderer *ren, RigidBall *ball, unsigned int winH, unsigned int winW, bool drawOriginLink);

double getPotEne(RigidState *state, double forceMat[][MAX_OBJS][DIMENSIONS]);
double getKinEne(RigidState *state);

void setInitials(RigidState *states[], int objCount);
void stepTime(RigidState *state[], double forceMat[][MAX_OBJS][DIMENSIONS], double dT, int objCount, double *simTime);
void odeSolve(double *fun, double *funDer, double dT, unsigned int id, unsigned char dimension);


#endif
