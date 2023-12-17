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

void printRigidBallState(RigidBall *ball);
void initRigidBall(RigidBall *ball, double radius, double mass);
void drawRigidBall(SDL_Renderer *ren, RigidBall *ball, unsigned int winH, unsigned int winW);
void odeSolve(RigidState *state, double forceMat[][MAX_OBJS][DIMENSIONS], double dT, int objCount);
