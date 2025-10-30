enum Rotation_Direction{
	LEFT,
	RIGHT
};


enum Motor_State{
	ENABLE,
	DISABLE
};

enum Homming_State{
	NOT_HOMMED = 10,
	HOMMED = 11
};


enum IS_PWM_ACTIVE{
	TIMER_NOT_ACTIVE = 2,
	CANAL_NOT_ACTIVE = 3,
	SETUP_NOT_ACTIVE = 4,
	PWM_ACTIVE = 5
};



//Values for calculation PWM freq
#define ARR_VALUE = 65535;
#define CLOCK_VALUE = 8000000;// 8 000 000
#define DUTY_CYCLE = 0.50;


//Movement constans
#define MINIMAL_SPEED = 5;
#define MAXIMAL_SPEED = 400;
#define MAX_ACCELERATION = 50;
#define MAX_DECELERATION = 50;
#define MAX_JERK = 100;
#define MINIMAL_SPEED_END_DISTANCE = 10;

//Hardware setup
#define STEPS_PER_REVOLUTION = 200;
#define MICROSTEPPING = 16;
