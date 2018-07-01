/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include "InfineonRacer.h"
#include "Basic.h"

/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/

#define GAIN_angleERROR_P 0.02
#define GAIN_angleERROR_D 0
#define GAIN_velERROR_P 1
#define Time 0.02

#define S_start			10.0
#define S_finish		117.0
#define S_center		63.5

/******************************************************************************/
/*--------------------------------Enumerations--------------------------------*/
/******************************************************************************/

typedef enum now_status{
	normal = 0,
	v_limit = 1,
	AEB = 2
}status_t;	// 주행 상태 구분

typedef enum scan_state{
	none = 11,
	right = 12,
	over_right = 13,
	big_over_right = 14,
	left = 21,
	middle = 22,
	over_left = 31,
	big_over_left = 41
}scan_state_t;	// Line scan camera state machine

/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*------------------------------Global variables------------------------------*/
/******************************************************************************/

InfineonRacer_t IR_Ctrl  /**< \brief  global data */
		={64, 64, FALSE  };


uint32 check_left;
uint32 check_right;
uint32 Find_Timecnt;
uint32 pixel_count_cross;
uint32 WhiteLane_cnt_left;
uint32 WhiteLane_cnt_right;

status_t status;
scan_state_t SCAN_STATE;


float Left0;
float Right0;
float Line_right;
float Line_left;
float error;
float pre_error;
float Angle;


/******************************************************************************/
/*-------------------------Function Prototypes--------------------------------*/
/******************************************************************************/
void set_scan_state(int check_left,int check_right);
void Find_Cross(void);
void get_error(void);
void InfineonRacer_ControlSrv(void);
/******************************************************************************/
/*------------------------Private Variables/Constants-------------------------*/
/******************************************************************************/

/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/
void InfineonRacer_init(void){
	status = normal;

	IR_setMotor0En(0.0);
	IR_Motor.Motor0Vol = 0.25;
	IR_getSrvAngle() = 0.0;

	WhiteLane_cnt_left = 0;
	WhiteLane_cnt_right = 0;
	Find_Timecnt = 0;

	Left0 = 25;
	Right0 = 102;

	error = 0;
	pre_error = 0;

}

void InfineonRacer_detectLane(void){

	check_left = 0;
	check_right = 0;
	pixel_count_cross = 0;

	for(uint32 pixel_left = 63; pixel_left >= S_start; pixel_left--)
	{
		if(IR_LineScan.adcResult[1][pixel_left] < 1200)
		{
			pixel_count_cross++;
			if(check_left == 0)
			{
				Line_left = pixel_left;
				check_left = 1;

			}
		}
	}

	for(uint32 pixel_right = 64; pixel_right < S_finish; pixel_right++)
	{
		if((IR_LineScan.adcResult[1][pixel_right] < 1200))
		{
			if(check_right == 0)
			{
				Line_right = pixel_right;
				check_right = 1;
			}
		}
	}

	if(status == v_limit)
	{
		if(check_right > check_left)   //값이 높은 곳이 하얀선
			WhiteLane_cnt_right++;

		else if(check_left > check_right)
			WhiteLane_cnt_left++;
	}

	set_scan_state(check_left,check_right);
	Find_Cross();

}


void InfineonRacer_control(void){
	float32  MotorVol;
	uint32 MotorVol_int;
	get_error();
	Angle  = (GAIN_angleERROR_P*error) + (GAIN_angleERROR_D*(error - pre_error)/Time);
	pre_error = error;

	InfineonRacer_ControlSrv();

	if(status == normal)
		{
			MotorVol_int = (int)((0.35 - GAIN_velERROR_P * Angle)*100);
			MotorVol = (float)MotorVol_int/100;
			IR_Motor.Motor0Vol = MotorVol;
		}

	if(status == v_limit)
		{
			MotorVol_int = (int)((0.15 - GAIN_velERROR_P * Angle)*100);
			MotorVol = (float)MotorVol_int/100;
			IR_Motor.Motor0Vol = MotorVol;
		}

}
/******************************************************************************/
/*----------------------------------------------------------------------------*/
/******************************************************************************/

void set_scan_state(int check_left,int check_right)
{
	if(((SCAN_STATE == 11) || (SCAN_STATE == 13) || (SCAN_STATE == 22)|| (SCAN_STATE == 12) ) && (check_left == 0) && (check_right == 1))
		SCAN_STATE = right;			//SCAN_STATE == 12

	else if(((SCAN_STATE == 12) || (SCAN_STATE == 14) || (SCAN_STATE == 13)) && (check_left == 1) && (check_right == 0))
		SCAN_STATE = over_right;	//SCAN_STATE == 13

	else if(((SCAN_STATE == 13)|| (SCAN_STATE == 14)) && (check_left == 0) && (check_right == 0))
		SCAN_STATE = big_over_right;		//SCAN_STATE == 14

	else if(((SCAN_STATE == 11) || (SCAN_STATE == 31) || (SCAN_STATE == 22) || (SCAN_STATE == 21)) && (check_left == 1) && (check_right == 0))
		SCAN_STATE = left;		//SCAN_STATE == 21

	else if(((SCAN_STATE == 21) || (SCAN_STATE == 41) || (SCAN_STATE == 31) ) &&(check_left == 0) && (check_right == 1))
		SCAN_STATE = over_left;		//SCAN_STATE == 31

	else if(((SCAN_STATE == 31) || (SCAN_STATE == 41)) && (check_left == 0) && (check_right == 0))
		SCAN_STATE = big_over_left;		//SCAN_STATE == 41

	else if (((SCAN_STATE == 11) || (SCAN_STATE == 12) || (SCAN_STATE == 21)|| (SCAN_STATE == 22)) && (check_left == 1) && (check_right == 1))
		SCAN_STATE = middle;		//SCAN_STATE == 22

	else if (((SCAN_STATE == 12) || (SCAN_STATE == 21) || (SCAN_STATE == 22)|| (SCAN_STATE == 11)) && (check_left == 0) && (check_right == 0))
		SCAN_STATE = none;		//SCAN_STATE == 11
}

void Find_Cross(void)
{

	Find_Timecnt++;

	if((pixel_count_cross >= 60) && (pixel_count_cross <= 100) && (Find_Timecnt >= 50))	// 0~128픽셀 중 흑색 픽셀이 50이상인 경우, status가 1초 주기로 변경된다.
	{
		if(status == normal)		// normal 상태이면
		{
			status = v_limit;		// 속도제한 구간에 들어온 상태로 변경
			IR_setBeeperOn(TRUE);	// Beep ON해서 알 수 있게 함.
			Find_Timecnt = 0;
		}

		else if(status == v_limit)		// v_limit 상태이면
		{
			status = normal;		// 속도제한 구간에서 나가는 상태
			IR_setBeeperOn(FALSE);	// Beep OFF로 알 수 있게 함.
			Find_Timecnt = 0;
		}

		if(Find_Timecnt >= 1000)
		{
			Find_Timecnt = 1000;
		}

	}
}

void get_error(void)
{

	if(SCAN_STATE == right)
		error = (Right0 - Line_right); 	//SCAN_STATE == 12 [offset = (S_center -(Right0 - Line_right))]

	else if(SCAN_STATE == over_right)
		error = (Right0 - Line_left);	  //SCAN_STATE == 13 [offset = (S_center -(Right0 - Line_left))]

	else if(SCAN_STATE == big_over_right)
		error  = S_center - S_start;		//SCAN_STATE == 14  [offset = S_start]

	else if(SCAN_STATE == left)
		error = (Left0 - Line_left);	//SCAN_STATE == 21	[offset = (S_center +(Line_left - Left0))]

	else if(SCAN_STATE == over_left)
		error = (Left0 - Line_right);	//SCAN_STATE == 31	[offset = (S_center +(Line_right - Right0))]

	else if(SCAN_STATE == big_over_left)
		error = S_center - S_finish;			//SCAN_STATE == 41	[offset = S_finish]

	else if (SCAN_STATE == middle)
		error = ((Line_right + Line_left)/2) - S_center;		//SCAN_STATE == 22 [offset =  (Line_right + Line_left)/2;]

	else if (SCAN_STATE == none)
		error = 0;							//SCAN_STATE == 11
}

void InfineonRacer_ControlSrv(void)   //error_value에 따라 서브모터의 angle 을 변형시킨다.
{//
	if( Angle < -0.2)
		IR_getSrvAngle() = -0.2;
	else if((Angle < -0.15) && (Angle >= -0.2))
		IR_getSrvAngle() = -0.15;
	else if((Angle < -0.10) && (Angle >= -0.15))
		IR_getSrvAngle() = -0.10;
	else if((Angle < -0.05) && (Angle >= -0.1))
		IR_getSrvAngle() = -0.05;

	else if((Angle <= 0.05) && (Angle >= -0.05))
		IR_getSrvAngle() = 0.0;

	else if((Angle > 0.05) && (Angle <= 0.1))
		IR_getSrvAngle() = 0.05;
	else if((Angle > 0.10) && (Angle <= 0.15))
		IR_getSrvAngle() = 0.10;
	else if((Angle > 0.15) && (Angle <= 0.2))
		IR_getSrvAngle() = 0.15;
	else if(Angle > 0.2)
		IR_getSrvAngle() = 0.2;
}


/******************************************************************************/
/*----------------------------- AVIOD  OBJECTS -------------------------------*/
/******************************************************************************/

void InfineonRacer_Avoid(sint32 task_cnt)
{
	float IR_Value;
	IR_Value = IR_getChn15();

	while(status == v_limit)
	{
		if(IR_Value >= 1.1)
		{
			//preIR_Value=IR_Value;
			if(WhiteLane_cnt_left > WhiteLane_cnt_right)
			{
					IR_getSrvAngle() = -0.1;
					IR_Value = IR_getChn15();
					task_cnt=0;
					while(task_cnt <= 200)
					{}

					//IR_getSrvAngle()= Kp*IR_Value + Kd*(IR_Value-preIR_Value)
					if((IR_Value < 1.1)&&(task_cnt>=200))
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

			else if(WhiteLane_cnt_right < WhiteLane_cnt_left)
			{
					IR_getSrvAngle() = 0.1;
					IR_Value = IR_getChn15();
					task_cnt=0;
					while(task_cnt <= 200)
					{}

					//IR_getSrvAngle()= Kp*IR_Value + Kd*(IR_Value-preIR_Value)
					if((IR_Value < 1.1)&&(task_cnt>=200))
					{
						IR_getSrvAngle() = -0.1;
						task_cnt=0;
						while(task_cnt <= 150)
						{}
					}

					if(task_cnt>=150)
					{
						IR_getSrvAngle() = 0.0;
					}			}
			WhiteLane_cnt_left = 0;
			WhiteLane_cnt_right = 0;
		}
	}
}

/* AEB Action */
void InfinedonRacer_AEB(void)
{
	float IR_AEB_Value;
	float AEB_speed = 0.4;
	IR_AEB_Value = IR_getChn15();

	while(status == AEB)
	{
		if(AEB_speed == 0)
			AEB_speed = 0;

		if(IR_AEB_Value >= 0.8)
		{
			AEB_speed = AEB_speed - 0.05;
		}
	}
}
