
// for stepper.h
#define MOTOR_STEPS 200
#define SLEEP 0
#define MS1 0
#define MS2 0
#define MS3 0
#define RPM 120


// global variables for the motor
long mspeed1 = 1000;
long mspeed2 = 1000;
long mspeed3 = 1000;
long mposition1 = 0;
long mposition2 = 0;
long mposition3 = 0;
boolean isstop = 0;

long POSITION_MOTOR_X = 0;
long POSITION_MOTOR_Y = 0;
long POSITION_MOTOR_Z = 0;

int MOTOR_ACCEL = 5000;
int MOTOR_DECEL = 5000;
  
int isabs = true;
int isblock = true;
int isen = false;

// direction
int SIGN_X = 1;
int SIGN_Y = 1;
int SIGN_Z = 1;

static const int FULLSTEPS_PER_REV_X = 200;
static const int FULLSTEPS_PER_REV_Y = 200;
static const int FULLSTEPS_PER_REV_Z = 200;