/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include "InfineonRacer.h"
#include "Basic.h"

/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/

#define GAIN_ANGLE_P 0.005	// 조향 제어 P Gain
#define GAIN_ANGLE_D 0		// 조향 제어 D Gain
#define GAIN_SPEED_P 1	// 속력 제어 P Gain
#define TIME_sampling 0.02	// Line Scan Camera Sampling Time

/* Line Scan Camera Constant values*/
#define S_START			3 	// 유용한 데이터의 시작
#define S_FINISH		117	// 유용한 데이터의 끝
#define S_CENTER		60		// 스캐너의 중앙 = 차량의 중앙
#define L_0	8					// 직선에 잘 정렬되었을 때 좌측 Line pixel
#define R_0	112					// 직선에 잘 정렬되었을 때 우측 Line pixel

/******************************************************************************/
/*--------------------------------Enumerations--------------------------------*/
/******************************************************************************/

typedef enum car_state{
	NORMAL = 0,
	V_LIMIT,
	AEB
}CAR_state_t;	// 주행 상태 구분

typedef enum dash_info{
	DASH_LEFT = 0,
	DASH_RIGHT
}DASHLINE_state_t;	// 점선 상태 구분

typedef enum scan_state{
	NONE = 11,
	RIGHT = 12,
	OVER_RIGHT = 13,
	BIG_OVER_RIGHT = 14,
	LEFT = 21,
	MIDDLE = 22,
	OVER_LEFT = 31,
	BIG_OVER_LEFT = 41
}SCAN_state_t;	// Line scan camera state machine

/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*------------------------------Global variables------------------------------*/
/******************************************************************************/

InfineonRacer_t IR_Ctrl  /**< \brief  global data */
		={64, 64, FALSE  };

CAR_state_t CAR_STATE;
DASHLINE_state_t DASH_STATE;
SCAN_state_t SCAN_STATE;

int OFFSET;			// OFFSET
int ERROR_steer;	// 차량의 중앙과 도로의 중앙의 pixel거리
int ERROR_pre;		// 이전 상태 Error
/******************************************************************************/
/*-------------------------Function Prototypes--------------------------------*/
/******************************************************************************/
void InfineonRacer_detectDashline(void);
void InfineonRacer_crosswalk(void);
void SET_STATUS(int LEFT_pixel, int RIGHT_pixel);
void SET_OFFSET(int Left_pixel, int Right_pixel);
int ACTION_STEER(int ERROR, int ERROR_pre);
/******************************************************************************/
/*------------------------Private Variables/Constants-------------------------*/
/******************************************************************************/

/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/
void InfineonRacer_init(void){
	CAR_STATE = NORMAL;
	SCAN_STATE = MIDDLE;

	IR_setMotor0En(0.0);
	IR_Motor.Motor0Vol = 0.25;
	IR_getSrvAngle() = 0.0;

	OFFSET = 60;
	ERROR_steer = 0;
}

void InfineonRacer_detectLane(void){

	int pixel;			// Scan을 위한 반복문 변수
	int Left_pixel;		// 좌측 라인의 pixel
	int Right_pixel; 	// 우측 라인의 pixel

	InfineonRacer_detectDashline();
	InfineonRacer_crosswalk();
	for(pixel = OFFSET; pixel >= S_START; pixel--)
	{
		if(IR_LineScan.adcResult[1][pixel] < 1200)
		{
			Left_pixel = pixel;
			break;
		}
	}

	for(pixel = OFFSET; pixel <= S_FINISH; pixel++)
	{
		if((IR_LineScan.adcResult[1][pixel] < 1200))
		{
			Right_pixel = pixel;
			break;
		}
	}

	SET_STATUS(Left_pixel, Right_pixel);
	SET_OFFSET(Left_pixel, Right_pixel);

	ERROR_pre = ERROR_steer;
	ERROR_steer = OFFSET - S_CENTER;
}

/********** DETECT DASH LINE IN SPEED LIMITED ZONE **********/
void InfineonRacer_detectDashline(void)
{
	int pixel;
	int DASH_LEFT_cnt;		// 좌측 점선 count 변수
	int DASH_RIGHT_cnt;		// 우측 점선 count 변수

	DASH_LEFT_cnt = 0;
	DASH_RIGHT_cnt = 0;

	if((CAR_STATE == V_LIMIT) && (SCAN_STATE == MIDDLE))
	{
		for(pixel = OFFSET; pixel >= S_START; pixel--)
		{
			if(IR_LineScan.adcResult[1][pixel] > 3000)
			{
				DASH_LEFT_cnt++;
			}
		}

		for(pixel = OFFSET; pixel <= S_FINISH; pixel++)
		{
			if((IR_LineScan.adcResult[1][pixel] > 3000))
			{
				DASH_RIGHT_cnt++;
			}
		}

		if(DASH_LEFT_cnt > DASH_RIGHT_cnt)
			DASH_STATE = LEFT;
		else
			DASH_STATE = RIGHT;
	}
}

/********** SEARCH CROSSWALK **********/
void InfineonRacer_crosswalk(void)
{
	int pixel;
	int BLACK_pixel;

	BLACK_pixel = 0;

	for(pixel = S_START; pixel >= S_FINISH; pixel++)
	{
		if(IR_LineScan.adcResult[1][pixel] < 1200)
		{
			BLACK_pixel++;
		}
	}

	if(BLACK_pixel >= 45)
	{
		switch (CAR_STATE){
		case NORMAL :
			CAR_STATE = V_LIMIT; break;
		case V_LIMIT :
			CAR_STATE = NORMAL; break;
		case AEB :
			CAR_STATE = AEB; break;
		}
	}
}

/********** SET STATE FUNCTION BASED ON STATE-MACHINE **********/
void SET_STATUS(int LEFT_pixel, int RIGHT_pixel)
{
	if((RIGHT_pixel > S_FINISH) && (LEFT_pixel < S_START))
	{
		SCAN_STATE = NONE; // state 11
	}

	else if((RIGHT_pixel >= S_CENTER) && (RIGHT_pixel <= S_FINISH) && (LEFT_pixel < S_START))
	{
		SCAN_STATE = RIGHT; // state 12
	}

	else if((RIGHT_pixel > S_START) && (RIGHT_pixel < S_CENTER) && (LEFT_pixel < S_START))
	{
		SCAN_STATE = OVER_RIGHT; // state 13
	}

	else if((RIGHT_pixel <= S_START) && (LEFT_pixel < S_START))
	{
		SCAN_STATE = BIG_OVER_RIGHT; // state 14
	}

	else if((RIGHT_pixel > S_FINISH) && (LEFT_pixel >= S_START) && (LEFT_pixel <= S_CENTER))
	{
		SCAN_STATE = LEFT; // state 21
	}

	else if((RIGHT_pixel >= S_CENTER) && (RIGHT_pixel <= S_FINISH) && (LEFT_pixel >= S_START) && (LEFT_pixel <= S_CENTER))
	{
		SCAN_STATE = MIDDLE; // state 22
	}

	else if((RIGHT_pixel > S_FINISH) && (LEFT_pixel > S_CENTER) && (LEFT_pixel < S_FINISH))
	{
		SCAN_STATE = OVER_LEFT; // state 31
	}

	else if((RIGHT_pixel > S_FINISH) && (LEFT_pixel >= S_FINISH))
	{
		SCAN_STATE = BIG_OVER_LEFT; // state 41
	}
}

/********** SET OFFSET FUNCTION ACCORDING TO EACH STATE **********/
void SET_OFFSET(int Left_pixel, int Right_pixel)
{
	switch (SCAN_STATE){

	case NONE :
		OFFSET = 64; break;
	case RIGHT :
		OFFSET = S_CENTER - (R_0 - Right_pixel); break;
	case OVER_RIGHT :
		OFFSET = S_CENTER - (R_0 - Right_pixel); if(OFFSET < S_START)OFFSET = S_START; break;
	case BIG_OVER_RIGHT :
		OFFSET = S_START; break;
	case LEFT :
		OFFSET = S_CENTER + (Left_pixel - L_0); break;
	case MIDDLE :
		OFFSET = (Left_pixel + Right_pixel) / 2; break;
	case OVER_LEFT :
		OFFSET = S_CENTER + (Left_pixel - L_0); if(OFFSET > S_FINISH) OFFSET = S_FINISH; break;
	case BIG_OVER_LEFT :
		OFFSET = S_FINISH; break;
	}
}

void InfineonRacer_control(void){
	float32 MotorVol;
	uint32 MotorVol_int;

	int Angle;

	if(CAR_STATE == NORMAL)
		{
			Angle = ACTION_STEER(ERROR_steer, ERROR_pre);
//			MotorVol_int = (int)((0.35 - GAIN_SPEED_P * Angle)*100);
			MotorVol = (0.35 - (GAIN_SPEED_P * Angle));
			IR_Motor.Motor0Vol = MotorVol;
		}

	if(CAR_STATE == V_LIMIT)
		{
			IR_Motor.Motor0Vol = 0.15;
		}

}

/********** ACTION TO CONTROL SERVO MOTOR FUNCTION **********/
int ACTION_STEER(int ERROR, int ERROR_pre)
{
	float ANGLE; 	// Input of Servo motor
	ANGLE = (GAIN_ANGLE_P * ERROR) - (GAIN_ANGLE_D * (ERROR-ERROR_pre)/TIME_sampling);

	/* LOAD PROTECTION */
	/* LEFT */
	if( ANGLE < -0.2)
		IR_getSrvAngle() = -0.2;
	else if((ANGLE < -0.15) && (ANGLE >= -0.2))
		IR_getSrvAngle() = -0.15;
	else if((ANGLE < -0.10) && (ANGLE >= -0.15))
		IR_getSrvAngle() = -0.10;
	else if((ANGLE < -0.05) && (ANGLE >= -0.1))
		IR_getSrvAngle() = -0.05;

	else if((ANGLE <= 0.05) && (ANGLE >= -0.05))
		IR_getSrvAngle() = 0.0;

	/* RIGHT */
	else if((ANGLE > 0.05) && (ANGLE <= 0.1))
		IR_getSrvAngle() = 0.05;
	else if((ANGLE > 0.10) && (ANGLE <= 0.15))
		IR_getSrvAngle() = 0.10;
	else if((ANGLE > 0.15) && (ANGLE <= 0.2))
		IR_getSrvAngle() = 0.15;
	else if(ANGLE > 0.2)
		IR_getSrvAngle() = 0.2;

	else
		IR_getSrvAngle() = 0.0;

	return (ANGLE);
}

/********** AVOID OBJECTS FUNCTION **********/
void InfineonRacer_Avoid(sint32 task_cnt)
{
	float IR_Value;		// IR Sensor Value
	IR_Value = IR_getChn15();

	if(CAR_STATE == V_LIMIT)
	{
		if(IR_Value >= 1.1)		// Distance <= 50cm
		{
			if(DASH_STATE == DASH_LEFT)
			{
				IR_getSrvAngle() = -0.1;
				task_cnt=0;
				while(task_cnt <= 200)
				{}

				if((IR_getChn15() < 1.1)&&(task_cnt>=200))
				{
					IR_getSrvAngle() = 0.1;
					task_cnt=0;
					while(task_cnt <= 150)
					{}
				}

				if(task_cnt>=150)
				{
					IR_getSrvAngle() = 0.0;
				}
			}

			else if(DASH_STATE == DASH_RIGHT)
			{
				IR_getSrvAngle() = 0.1;
				task_cnt=0;
				while(task_cnt <= 200)
				{}

				if((IR_getChn15() < 1.1)&&(task_cnt>=200))
				{
					IR_getSrvAngle() = -0.1;
					task_cnt=0;
					while(task_cnt <= 150)
					{}
				}

				if(task_cnt>=150)
				{
					IR_getSrvAngle() = 0.0;
				}
			}
		}
	}
}

/********** AEB ACTION **********/
//void InfinedonRacer_AEB(void)
//{
//	float IR_AEB_Value;
//	float AEB_speed = 0.35;
//	IR_AEB_Value = IR_getChn15();
//
//	if(CAR_STATE == AEB)
//	{
//		if(AEB_speed == 0)
//			AEB_speed = 0;
//
//		if(IR_AEB_Value >= 0.8)
//		{
//			AEB_speed = AEB_speed - 0.05;
//		}
//	}
//}
