#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "matrixOps.h"
#include "jacobians.h"

#define PIXELS_PER_METER 400
#define X_OFFSET 0
#define Y_OFFSET 0
#define BALL_SIZE 20
#define MAX_OBJS 5
#define DIMENSIONS 2

typedef enum{
	NONE,   
    RADIUS,
} ValueType;

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

	bool fixed; //Determines if the current state has to be allowed to move

} RigidState;

typedef struct tmpConstraint {
    void (*propagateForces)(double forceMat[][MAX_OBJS][DIMENSIONS], struct tmpConstraint *cons); //Force propagation between the internal states
    void (*updateJacobians)(Matrix *jacob, Matrix *jacob2, Matrix *corrector, int currIndex, struct tmpConstraint *cons, RigidState *states[MAX_OBJS]); //Computes the jacobians and their time derivative and the trajectory deviation, currIndex indicates the index of the row of the jacobian matrix where to put data, will be equal to the current constraint index
	//void (*fillMatrix)(double forceMat[][MAX_OBJS][DIMENSIONS], Constraint *cons, RigidState *states[], Matrix *force, Matrix *velocity, Matrix *mass, int currIndex);

    double value; //Can have multiple meanings based on the type of the constraint (bad decision, however i'm dumb and this is all I can come up with)
    ValueType valType;

    uint32_t stateAIndex;
    uint32_t stateBIndex;

} Constraint;

typedef struct{
	uint32_t x;
	uint32_t y;
	uint32_t color;
	float radius;
} SDL_PointColor;

//Rigid body definitions
typedef struct {
	double radius;
	RigidState state;
} RigidBall;

typedef struct {
	RigidState stateA;
	RigidState stateB;
	Constraint constraint;
	double lenght;
	double thickness; //Optional

} RigidBeam;



void initRigidBall(RigidBall *ball, double radius, double mass);
bool initRigidBeam(RigidBeam *beam, double lenght, double mass, double thickness);
void getForceConstraint(double forceMat[][MAX_OBJS][DIMENSIONS], Constraint *cons, RigidState *states[]);
void initHingeConstraint(Constraint *constraint, uint32_t stateAInd, uint32_t stateBInd);
void fillMatrices(double forceMat[][MAX_OBJS][DIMENSIONS], Constraint *cons, RigidState *states[], Matrix *force, Matrix *velocity, Matrix *mass, int currIndex);
void printState(RigidState *state, double forceMat[][MAX_OBJS][DIMENSIONS]);

void drawLink(SDL_Renderer *ren, RigidState *ball, RigidState *ball2, unsigned int winH, unsigned int winW);
void drawTraj(SDL_Renderer *ren, RigidState *state, SDL_Texture *trajHandle);
void drawRigidBall(SDL_Renderer *ren, RigidBall *ball, unsigned int winH, unsigned int winW, bool drawOriginLink);

bool registerState(RigidState *state);
bool registerConstraint(Constraint *cons);

void printDistance(RigidState *A, RigidState *B);
double getPotEne(RigidState *state, double forceMat[][MAX_OBJS][DIMENSIONS]);
double getKinEne(RigidState *state);

void setInitials(RigidState *states[], int objCount);
void stepSystemTime(RigidState *state[], double forceMat[][MAX_OBJS][DIMENSIONS], double dT, int objCount, double *simTime);
void odeSolve(double *fun, double *funDer, double dT, unsigned int id, unsigned char dimension);

extern RigidState *globState[MAX_OBJS];
extern Constraint *globConst[MAX_OBJS];
extern uint32_t stateIndex, constIndex;

#endif
