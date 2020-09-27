/**
 ******************************************************************************
 * @file       example.c
 * @date       01/10/2014 12:00:00
 * @brief      Example functions for the X-NUCLEO-IHM02A1
 ******************************************************************************
 *
 * COPYRIGHT(c) 2014 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#include "main.h"
#include "example.h"
#include "example_usart.h"
#include "params.h"
#include "xnucleoihm02a1_interface.h"
#include "xnucleoihm02a1.h"

extern TIM_HandleTypeDef htim2;
extern uint8_t TIM2_AWAITING_UPDATE_EVENT;

static void TimerStart(double Ts);
static void TimerStop(void);

static uint32_t PCLK1TIM(void);
static void wait(double Ts);
static void GoTo(int32_t  X, int32_t  Y);
static void Move(int32_t DX, int32_t DY);

static int32_t gcd(int32_t a, int32_t b);
static void MoveBy(int32_t DX_target, int32_t DY_target);

StepperMotorBoardHandle_t *StepperMotorBoardHandle;
MotorParameterData_t *MotorParameterDataGlobal, *MotorParameterDataSingle;

/**
 * @addtogroup MicrosteppingMotor_Example
 * @{
 */

/**
 * @addtogroup Example
 * @{
 */

/**
 * @defgroup   ExamplePrivateFunctions
 * @brief      Example Private Functions.
 * @{
 */

uint16_t BSP_ST1S14_PGOOD(void);
uint32_t usrPow(uint8_t base, uint8_t exponent);

/**
 * @}
 */ /* End of ExamplePrivateFunctions */

/**
 * @addtogroup ExamplePrivateFunctions
 * @brief      Example Private Functions.
 * @{
 */

/**
 * @addtogroup ExampleExportedFunctions
 * @brief      Example Exported Functions.
 * @{
 */

/**
 * @brief  Example no.1 for X-NUCLEO-IHM02A1.
 * @note	Perform a complete motor axis revolution as MPR_1 equal movements,
 *			for each L6470 mounted on all stacked X-NUCLEO-IHM02A1.
 *			At the end of each movement there is a delay of DELAY_1 ms.
 *     	After each motor has performed a complete revolution there is a
 *			delay of DELAY_2 ms.
 *			Now all motors for each X-NUCLEO-IHM02A1 will start at the same
 *			time.
 *			They are going to run at INIT_SPEED for DELAY_3 ms.
 *			After that all motors for each X-NUCLEO-IHM02A1 will get a HardStop
 *			at the same time.
 *			Perform a complete motor axis revolution as MPR_2 equal movements,
 *			for each L6470 mounted on all stacked X-NUCLEO-IHM02A1.
 *			At the end of each movement there is a delay of DELAY_1 ms.
 *			After that all motors for each X-NUCLEO-IHM02A1 will get a HardHiZ
 *			at the same time.
 */
void MicrosteppingMotor_Example_01(void)
{
	sL6470_ConfigRegister_t t;

#define MPR_1     4			  //!< Motor Movements Per Revolution 1st option
#define MPR_2     8			  //!< Motor Movements Per Revolution 2nd option
#define DELAY_1   1000		//!< Delay time 1st option
#define DELAY_2   2500		//!< Delay time 2nd option
#define DELAY_3   10000   //!< Delay time 3rd option

	uint32_t Speed0=0, Speed1=0, Acc=0, Dec=0;
	uint32_t MaxSpeed0=0, MaxSpeed1=0;
	float speed, acc, dec;
	//	uint8_t MovementPerRevolution;
	uint8_t board=0;
	uint8_t device;

#ifdef NUCLEO_USE_USART
	USART_Transmit(&huart2, (uint8_t *)"Initial values for registers:\r\n");
	USART_PrintAllRegisterValues();
#endif

	/* Setup each X-NUCLEO-IHM02A1 Expansion Board ******************************/

	/*
typedef struct {
  float     motorvoltage;           //!< motor supply voltage in V
  float     fullstepsperrevolution; //!< min number of steps per revolution for the motor
  float     phasecurrent;           //!< max motor phase voltage in A
  float     phasevoltage;           //!< max motor phase voltage in V

  float     speed;                  //!< motor initial speed [step/s]
  float     acc;                    //!< motor acceleration [step/s^2] (comment for infinite acceleration mode)
  float     dec;                    //!< motor deceleration [step/s^2] (comment for infinite deceleration mode)
  float     maxspeed;               //!< motor maximum speed [step/s]
  float     minspeed;               //!< motor minimum speed [step/s]

  float     fsspd;                  //!< motor full-step speed threshold [step/s]
  float     kvalhold;               //!< holding kval [V]
  float     kvalrun;                //!< constant speed kval [V]
  float     kvalacc;                //!< acceleration starting kval [V]
  float     kvaldec;                //!< deceleration starting kval [V]
  float     intspeed;               //!< intersect speed for bemf compensation curve slope changing [step/s]
  float     stslp;                  //!< start slope [s/step]
  float     fnslpacc;               //!< acceleration final slope [s/step]
  float     fnslpdec;               //!< deceleration final slope [s/step]
  uint8_t   kterm;                  //!< thermal compensation factor (range [0, 15])
  float     ocdth;                  //!< ocd threshold [ma] (range [375 ma, 6000 ma])
  float     stallth;                //!< stall threshold [ma] (range [31.25 ma, 4000 ma])
  uint8_t   step_sel;               //!< step mode selection
  uint8_t   alarmen;                //!< alarm conditions enable
  uint16_t  config;                 //!< ic configuration
}MotorParameterData_t;
	 const MotorParameterData_t MotorParameterInitData[EXPBRD_MOUNTED_NR_MAX][L6470DAISYCHAINSIZE] = {
  {
    {9.0, 400, 1.7, 3.06, \
    240.0, 400.0, 400.0, 320.0, 0.0, \
    602.7, 3.06, 3.06,\
      3.06, 3.06, 61.52, 392.1569e-6, 643.1372e-6, 643.1372e-6, 0,\
        3.06*1000*1.10, 3.06*1000*1.00, MICROSTEP_1_128, 0xFF, 0x2E88},
    {9.0, 400, 1.7, 3.06, 240.0, 400.0, 400.0, 320.0, 0.0, 602.7, 3.06, 3.06,\
      3.06, 3.06, 61.52, 392.1569e-6, 643.1372e-6, 643.1372e-6, 0,\
        3.06*1000*1.10, 3.06*1000*1.00, MICROSTEP_1_128, 0xFF, 0x2E88},
  }, ...
	 */
	/*
	MotorParameterData_t:

	{20.0, 200, 1.7, 3.06, \
	1400.0, 1500.0, 1500.0, 1400.0, 0.0, \
	1602.7, 3.06, 3.06, \
	3.06, 3.06, 61.52, 392.1569e-6, 643.1372e-6, 643.1372e-6, 0,
	3.06*1000*1.10, 3.06*1000*1.00, MICROSTEP_1_128, 0xFF, 0x1E88},

	{motorvoltage, fullstepsperrevolution, phasecurrent, phasevoltage, speed, acc, dec, maxspeed, minspeed, fsspd, kvalhold, kvalrun,
	kvalacc, kvaldec, intspeed, stslp, fnslpacc, fnslpdec, kterm,
	ocdth, stallth, step_sel, alarmen, config}

	-------------------------------------------
	NUCLEO-F401RE, X-NUCLEO-IHM02A1,

	L6470: Fully integrated microstepping motor driver with motion engine and SPI

	Stepper Motor: SANYO DENKI - SANMOTION 103H5210-5240 Stepper Motor
	 */

	/* Get the parameters for the motor connected with the 1st stepper motor driver of the 1st stepper motor expansion board */
	MotorParameterDataGlobal = GetMotorParameterInitData();
	printf("%% MotorParameterDataGlobal Config : %x \n", MotorParameterDataGlobal->config);

//	L6470_SetParam(L6470_ID(0), L6470_CONFIG_ID, 0x1E85);
//	L6470_SetParam(L6470_ID(1), L6470_CONFIG_ID, 0x1E85);
//	uint32_t configReg0=L6470_GetParam(L6470_ID(0), L6470_CONFIG_ID);
//	uint32_t configReg1=L6470_GetParam(L6470_ID(1), L6470_CONFIG_ID);
//	printf("%% ConfigReg0 : %x  configReg1 : %x \n", configReg0, configReg1);

	StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(0);
	MotorParameterDataSingle = MotorParameterDataGlobal;
	StepperMotorBoardHandle->Config(MotorParameterDataSingle);

	uint32_t configReg0=L6470_GetParam(L6470_ID(0), L6470_CONFIG_ID);
	uint32_t configReg1=L6470_GetParam(L6470_ID(1), L6470_CONFIG_ID);
	printf("%% Checking *** ConfigReg0 : %x  configReg1 : %x \n", configReg0, configReg1);
	if (configReg0 !=  0x1E85) Error_Handler();
	if (configReg1 !=  0x1E85) Error_Handler();

//	L6470_SetParam(L6470_ID(0), L6470_CONFIG_ID, 0x1E85);
//	L6470_SetParam(L6470_ID(1), L6470_CONFIG_ID, 0x1E85);
//	configReg0=L6470_GetParam(L6470_ID(0), L6470_CONFIG_ID);
//	configReg1=L6470_GetParam(L6470_ID(1), L6470_CONFIG_ID);
//	printf("%% After  *** ConfigReg0 : %x  configReg1 : %x \n", configReg0, configReg1);


#ifdef NUCLEO_USE_USART
	USART_Transmit(&huart2, (uint8_t *)"Custom values for registers:\r\n");
	USART_PrintAllRegisterValues();
#endif

	/****************************************************************************/

	BSP_L6470_ResetPos(0, L6470_ID(0));
	BSP_L6470_ResetPos(0, L6470_ID(1));

	int32_t X=AbsPos_2_Position(L6470_GetParam(L6470_ID(0), L6470_ABS_POS_ID));
	int32_t Y=AbsPos_2_Position(L6470_GetParam(L6470_ID(1), L6470_ABS_POS_ID));
	printf("%% Posx : %d[step]  Posy : %d[step] \n", X, Y);

	//	//#define STEPCLOCK
	//	#ifdef STEPCLOCK
	//
	//    //* @warning Setting the step-clock mode implies first disabling the power bridge through
	//    //*          the soft_hiz() method.
	//
	//	/* Prepare the stepper driver to be ready to perform the SoftHiZ command
	//	 * which disables the power bridges (high impedance state). */
	//	device=L6470_ID(0);
	//	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareHardHiZ(device);
	//
	//	device=L6470_ID(1);
	//	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareHardHiZ(device);
	//
	//	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();
	//
	//	for (device = L6470_ID(0); device <= L6470_ID(1); device++)
	//	{
	//		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board, device, BUSY_ID) == 0);
	//	}
	//
	//
	//	device=L6470_ID(0);
	//	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareStepClock(device, L6470_DIR_FWD_ID);
	//
	//	device=L6470_ID(1);
	//	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareStepClock(device, L6470_DIR_FWD_ID);
	//
	//	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();
	//
	//	for (device = L6470_ID(0); device <= L6470_ID(1); device++)
	//	{
	//		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board, device, BUSY_ID) == 0);
	//	}
	//
	//	for (int n=0; n<400; n++){
	//		HAL_GPIO_TogglePin(L6470_STCK0_GPIO_Port, L6470_STCK0_Pin);
	//		HAL_GPIO_TogglePin(L6470_STCK1_GPIO_Port, L6470_STCK1_Pin);
	//		HAL_Delay(2);
	//		HAL_GPIO_TogglePin(L6470_STCK1_GPIO_Port, L6470_STCK1_Pin);
	//		HAL_Delay(2);
	////		TimerStart(1e-3);
	////		while (TIM2_AWAITING_UPDATE_EVENT) {;}
	////		TimerStop();
	//
	////		HAL_GPIO_TogglePin(L6470_STCK1_GPIO_Port, L6470_STCK1_Pin);
	////		TimerStart(1e-3);
	////		while (TIM2_AWAITING_UPDATE_EVENT) {;}
	////		TimerStop();
	//
	////#define USART
	//#ifdef USART
	//		X=AbsPos_2_Position(L6470_GetParam(L6470_ID(0), L6470_ABS_POS_ID));
	//		Y=AbsPos_2_Position(L6470_GetParam(L6470_ID(1), L6470_ABS_POS_ID));
	//		printf("Posx : %d[step]  Posy : %d[step] \n", X, Y);
	//#endif
	//	}
	//
	//#else
	//
	//	for (int n=0; n<200; n++){
	//		/*
	//		device=L6470_ID(0);
	//		L6470_Move(device, L6470_DIR_FWD_ID, 1);
	//		device=L6470_ID(1);
	//		L6470_Move(device, L6470_DIR_FWD_ID, 1);
	//		HAL_Delay(2);
	//		*/
	//		device=L6470_ID(0);
	//		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareMove(device, L6470_DIR_FWD_ID, 1);
	//
	//		device=L6470_ID(1);
	//		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareMove(device, L6470_DIR_FWD_ID, 10);
	//
	//		StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();
	//
	//		for (device = L6470_ID(0); device <= L6470_ID(1); device++)
	//		{
	//			while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board, device, BUSY_ID) == 0);
	//		}
	//		#define USART
	//		#ifdef USART
	//		X=AbsPos_2_Position(L6470_GetParam(L6470_ID(0), L6470_ABS_POS_ID));
	//		Y=AbsPos_2_Position(L6470_GetParam(L6470_ID(1), L6470_ABS_POS_ID));
	//		printf("%d Posx : %d[step]  Posy : %d[step] \n", n, X, Y);
	//		#endif
	//	}
	//
	//#endif
	//
	//	X=AbsPos_2_Position(L6470_GetParam(L6470_ID(0), L6470_ABS_POS_ID));
	//	Y=AbsPos_2_Position(L6470_GetParam(L6470_ID(1), L6470_ABS_POS_ID));
	//	printf("Posx : %d[step]  Posy : %d[step] \n", X, Y);
	//
	//	device=L6470_ID(0);
	//	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareGoHome(device);
	//
	//	device=L6470_ID(1);
	//	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareGoHome(device);
	//
	//	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();
	//
	//	for (device = L6470_ID(0); device <= L6470_ID(1); device++)
	//	{
	//		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board, device, BUSY_ID) == 0);
	//	}
	//	while(1) { ; }

	printf("%% ===================================\n");

	//	float accMax=10000.;
	//		Acc = Step_s2_2_Acc(accMax);
	//		L6470_SetParam(L6470_ID(0), L6470_ACC_ID, Acc);
	//		L6470_SetParam(L6470_ID(1), L6470_ACC_ID, Acc);
	//
	//		Dec = Step_s2_2_Acc(accMax);
	//		L6470_SetParam(L6470_ID(0), L6470_DEC_ID, Dec);
	//		L6470_SetParam(L6470_ID(1), L6470_DEC_ID, Dec);
	//
	////#define ASYNC
	//    uint32_t DX=100, DY=50;
	//	for (int nstep=0; nstep<10000; nstep++){
	//#ifdef ASYNC
	//		device=L6470_ID(0);
	//		L6470_Move(device, L6470_DIR_FWD_ID, DX);
	//
	//		device=L6470_ID(1);
	//		L6470_Move(device, L6470_DIR_FWD_ID, DY);
	//#else
	//		device=L6470_ID(0);
	//		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareMove(device, L6470_DIR_FWD_ID, DX);
	//
	//		device=L6470_ID(1);
	//		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareMove(device, L6470_DIR_FWD_ID, DY);
	//
	//		StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();
	//
	//		for (device = L6470_ID(0); device <= L6470_ID(1); device++)
	//		{
	//			while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board, device, BUSY_ID) == 0);
	//		}
	//#endif
	//		for (device = L6470_ID(0); device <= L6470_ID(1); device++)
	//		{
	//			/* Prepare the stepper driver to be ready to perform the HardStop command
	//			 * which causes an immediate motor stop with infinite deceleration */
	//			StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareHardStop(device);
	//		}
	//
	//		StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();
	//
	//		for (device = L6470_ID(0); device <= L6470_ID(1); device++)
	//		{
	//			while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board, device, BUSY_ID) == 0);
	//		}

	//		int32_t X=AbsPos_2_Position(L6470_GetParam(L6470_ID(0), L6470_ABS_POS_ID));
	//		int32_t Y=AbsPos_2_Position(L6470_GetParam(L6470_ID(1), L6470_ABS_POS_ID));
	//
	//		//		printf("Posx : %d[step]  Posy : %d[step] %f %f\n", X, Y, speedX, speedY);
	//		printf("%d %d\n", X, Y);
	//	}
	//	while(1) {; }


	uint32_t Step_2_uStep = 128;
	uint32_t Xtarget= 100000; // MICROSTEP_1_128
	uint32_t Ytarget= 100000; // MICROSTEP_1_128

	float theta=atan2(Ytarget, Xtarget);

	float speedMax=2000.0; //1500.0;
	float accMax  =1500.0; //1500.0;
	float dLim = speedMax*speedMax/accMax;
	if (dLim*Step_2_uStep<(float) Xtarget)
		while(1) { ; }
	if (dLim*Step_2_uStep<(float) Ytarget)
		while(1) { ; }

	float fsspd =  1602.7;
	uint32_t Fsspd=Step_s_2_FsSpd(fsspd);
	uint32_t Acc0, Acc1;
	uint32_t Dec0, Dec1;

	speed=speedMax;
	uint32_t MaxSpeed = Step_s_2_MaxSpeed(speed);

	float accX, accY;

	accX=accMax*cos(theta); // ACHTUNG
	accY=accMax*sin(theta);

	Acc0 = Step_s2_2_Acc(accX);
	Acc1 = Step_s2_2_Acc(accY);
	Dec0=Acc0;
	Dec1=Acc1;

//	if (tan(theta)<1.0){
//		accX=accMax;
//		Acc0 = Step_s2_2_Acc(accX*1.0288); // ACHTUNG
//		accY=accMax*tan(theta);
//		Acc1 = Step_s2_2_Acc(accY);
//	}
//	else{
//		accX=accMax/tan(theta);
//		Acc0 = Step_s2_2_Acc(accX*1.0288); // ACHTUNG
//		accY=accMax;
//		Acc1 = Step_s2_2_Acc(accY);
//	}
//
//	Dec0=Acc0;
//	Dec1=Acc1;

	float tauX = sqrt((float)Xtarget/Step_2_uStep/accX);
	float tauY = sqrt((float)Ytarget/Step_2_uStep/accY);
    float tau = (tauX>tauY? tauX: tauY);

    printf("%% tau %f tauX %f tauY %f\n", tau, tauX, tauY);

	device=L6470_ID(0);
	//Acc0=0xffe; // MAX ACC
	//acc=Acc_2_Step_s2(Acc0);
	L6470_SetParam(device, L6470_ACC_ID, Acc0);
	L6470_SetParam(device, L6470_DEC_ID, Dec0);
	L6470_SetParam(device, L6470_MIN_SPEED_ID, 0);
	L6470_SetParam(device, L6470_MAX_SPEED_ID, MaxSpeed);
	L6470_SetParam(device, L6470_FS_SPD_ID, Fsspd);

	device=L6470_ID(1);
	// Acc1=0xffe;
	//Dec1=0xffe;
	L6470_SetParam(device, L6470_ACC_ID, Acc1);
	L6470_SetParam(device, L6470_DEC_ID, Dec1);
	L6470_SetParam(device, L6470_MIN_SPEED_ID, 0);
	L6470_SetParam(device, L6470_MAX_SPEED_ID, MaxSpeed);
	L6470_SetParam(device, L6470_FS_SPD_ID, Fsspd);

	/*=================================================================*/



device=L6470_ID(0);
#define GOTO
#ifdef GOTO
	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareGoTo(device, Xtarget);
#else
	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareMove(device, L6470_DIR_FWD_ID, Xtarget);
#endif

	device=L6470_ID(1);
#ifdef GOTO
	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareGoTo(device, Ytarget);
#else
	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareMove(device, L6470_DIR_FWD_ID, Ytarget);
#endif

	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

	//		wait(10000);

	{
		float Ts=2.0*tau;
		uint32_t PCLK1TIM_FREQ=PCLK1TIM();
		uint32_t PSC=TIM2->PSC;
		uint32_t FREQ = PCLK1TIM_FREQ/(PSC+1);
		uint32_t CNT = (uint32_t)(FREQ* Ts)-1;

		__HAL_TIM_SET_AUTORELOAD(&htim2, CNT);
		HAL_TIM_Base_Start_IT(&htim2);
		TIM2_AWAITING_UPDATE_EVENT=1;
		while (TIM2_AWAITING_UPDATE_EVENT){
			int32_t X=AbsPos_2_Position(L6470_GetParam(L6470_ID(0), L6470_ABS_POS_ID));
			int32_t Y=AbsPos_2_Position(L6470_GetParam(L6470_ID(1), L6470_ABS_POS_ID));
			int32_t SpeedX=L6470_GetParam(L6470_ID(0), L6470_SPEED_ID);
			int32_t SpeedY=L6470_GetParam(L6470_ID(1), L6470_SPEED_ID);
			float speedX = Speed_2_Step_s(SpeedX);
			float speedY = Speed_2_Step_s(SpeedY);
			uint32_t count=__HAL_TIM_GET_COUNTER(&htim2);
			float t=(count+1.0f)/(float)FREQ;
			//		printf("Posx : %d[step]  Posy : %d[step] %f %f\n", X, Y, speedX, speedY);
			printf("%f %d %d %f %f\n", t, X, Y, speedX, speedY);
		}
	}
	HAL_TIM_Base_Stop_IT(&htim2);

	//	HAL_Delay(10000);

	for (device = L6470_ID(0); device <= L6470_ID(1); device++)
	{
		/* Prepare the stepper driver to be ready to perform the HardStop command
		 * which causes an immediate motor stop with infinite deceleration */
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareHardStop(device);
	}

	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

	for (device = L6470_ID(0); device <= L6470_ID(1); device++)
	{
		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board, device, BUSY_ID) == 0);
	}

	while(1);
	//	for (device = L6470_ID(0); device <= L6470_ID(1); device++)
	//	{
	//		//		/* Get the parameters for the motor connected with the actual stepper motor driver of the actual stepper motor expansion board */
	//		//		MotorParameterDataSingle = MotorParameterDataGlobal+(device);
	//		//		N_STEPS = ((uint32_t)MotorParameterDataSingle->fullstepsperrevolution * usrPow(2, MotorParameterDataSingle->step_sel)) / MovementPerRevolution;
	//		//
	//		//		/* Prepare the stepper driver to be ready to perform the Move command
	//		//		 * which produces a motion of N_STEPS microsteps */
	//		//		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareMove(device, DIR, N_STEPS*device+N_STEPS);
	//
	//		/* Set Speed */
	//		float speed = MotorParameterDataSingle->maxspeed*(1.+(float)device)/2.*0.1;
	//		Speed = Step_s_2_Speed(speed);
	//		printf("L6470_ID(%d) Speed : %f [step/s]  ==> %lu\n", device, speed, Speed);
	//
	//
	//		/* Prepare the stepper driver to be ready to perform the Run command which produces a motion at fixed speed.
	//		 * The speed value is in (([step/s] * 250e-9) / 2^-28) unit */
	////		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareRun(device, L6470_DIR_REV_ID, Speed);
	//		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareGoTo(device, 100000*(device+1));
	////		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareGoUntil(device, L6470_ACT_CPY_ID, L6470_DIR_FWD_ID, Speed);
	//	}
	//
	//	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();
	//
	//	for (device = L6470_ID(0); device <= L6470_ID(1); device++)
	//	{
	//		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board, device, BUSY_ID) == 0);
	//	}
	//
	//	//HAL_Delay(DELAY_2);
	//	HAL_Delay(1000);
	//
	//	for (device = L6470_ID(0); device <= L6470_ID(1); device++)
	//	{
	//		/* Prepare the stepper driver to be ready to perform the HardStop command
	//		 * which causes an immediate motor stop with infinite deceleration */
	//		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareHardStop(device);
	//	}
	//
	//	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();
	//
	//	for (device = L6470_ID(0); device <= L6470_ID(1); device++)
	//	{
	//		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board, device, BUSY_ID) == 0);
	//	}

	HAL_Delay(DELAY_2);


	for (device = L6470_ID(0); device <= L6470_ID(1); device++)
	{
		/* Prepare the stepper driver to be ready to perform the GoHome command */
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareGoHome(device);
	}

	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

	for (device = L6470_ID(0); device <= L6470_ID(1); device++)
	{
		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board, device, BUSY_ID) == 0);
	}

	//HAL_Delay(DELAY_3);

	for (device = L6470_ID(0); device <= L6470_ID(1); device++)
	{
		/* Prepare the stepper driver to be ready to perform the HardStop command
		 * which causes an immediate motor stop with infinite deceleration */
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareHardStop(device);
	}

	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

	for (device = L6470_ID(0); device <= L6470_ID(1); device++)
	{
		/* Prepare the stepper driver to be ready to perform the HardHiZ command
		 * immediately disables the power bridges (high impedance state). */
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareHardHiZ(device);
	}

	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

	/* Switch on the user LED */
	BSP_LED_On(LED2);
}

/**
 * @}
 */ /* End of ExampleExportedFunctions */

/**
 * @brief  This function return the ADC conversion result about the ST1S14 PGOOD.
 * @retval PGOOD The number into the range [0, 4095] as [0, 3.3]V.
 * @note   It will just return 0 if USE_ST1S14_PGOOD is not defined.
 */
uint16_t BSP_ST1S14_PGOOD(void)
{
#ifdef USE_ST1S14_PGOOD
	HAL_ADC_Start(&HADC);
	HAL_ADC_PollForConversion(&HADC, 100);

	return HAL_ADC_GetValue(&HADC);
#else
	return 0;
#endif
}

/**
 * @brief  Calculates the power of a number.
 * @param  base      the base
 * @param  exponent  the exponent
 * @retval power     the result as (base^exponent)
 * @note   There is not OVF control.
 */
uint32_t usrPow(uint8_t base, uint8_t exponent)
{
	uint8_t i;
	uint32_t power = 1;

	for (i=0; i<exponent; i++)
		power *= base;

	return power;
}

/**
 * @brief  EXTI line detection callbacks.
 * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
	case GPIO_PIN_13:
		BSP_EmergencyStop();
		break;
	case L6470_nBUSY_SYNC_Pin:
		BSP_L6470_BusySynchEventManager();
		break;
	case L6470_nFLAG_Pin:
		BSP_L6470_FlagEventManager();
		break;
	}
}



//#define PREPAREMOVE
#define HARDSTOP

/**
 * @brief  goto command
 * @param  signed int X, Y in [-2**21, 2**21-1]=[-2097152, 2097151]
 * @retval None
 */
static void GoTo(int32_t X, int32_t Y){
	uint8_t device;

	/* Prepare the stepper driver to be ready to perform the GoTo command which produces
	 *  a motion to ABS_POS absolute position through *** the shortest path *** */
	device=L6470_ID(0);
	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareGoTo(device, Position_2_AbsPos(X));

	/* Prepare the stepper driver to be ready to perform the GoTo command which produces
	 *  a motion to ABS_POS absolute position through *** the shortest path *** */
	device=L6470_ID(1);
	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareGoTo(device, Position_2_AbsPos(Y));

	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

	for (uint8_t device = L6470_ID(0); device <= L6470_ID(1); device++)
	{
		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(0, device, BUSY_ID) == 0){
			//uint32_t Speed = L6470_GetParam(device, L6470_SPEED_ID);
			//printf("device %d V=%lu Speed %f\n", device, BSP_ST1S14_PGOOD(), Speed_2_Step_s(Speed));
			//printf("%f\n", Speed_2_Step_s(Speed));
		}
	}

	X=AbsPos_2_Position(L6470_GetParam(L6470_ID(0), L6470_ABS_POS_ID));
	Y=AbsPos_2_Position(L6470_GetParam(L6470_ID(1), L6470_ABS_POS_ID));
	printf("Posx : %d[step]  Posy : %d[step] \n", X, Y);

	for (uint8_t device = L6470_ID(0); device <= L6470_ID(1); device++)
	{
		/* Prepare the stepper driver to be ready to perform the HardStop command
		 * which causes an immediate motor stop with infinite deceleration */
#ifdef HARDSTOP
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareHardStop(device);
#else
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareSoftStop(device);
#endif
	}

	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

	for (uint8_t device = L6470_ID(0); device <= L6470_ID(1); device++)
	{
		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(0, device, BUSY_ID) == 0);
	}
}

static void Move(int32_t DX, int32_t DY){
	uint8_t device;
	eL6470_DirId_t DIRX, DIRY;
	float dx=(float) DX;
	float dy=(float) DY;
	float dr=sqrt(dx*dx+dy*dy);
	float ct=dx/dr;
	float st=dy/dr;

	device=L6470_ID(0);
	if (ct > 0)
		DIRX = L6470_DIR_FWD_ID;
	else
		DIRX = L6470_DIR_REV_ID;

	/* Prepare the stepper driver to be ready to perform the Move command which produces
	 * a motion of abs(DX) microsteps along the DIRX direction*/
	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareMove(device, DIRX , abs(DX));

	device=L6470_ID(1);
	if (st > 0)
		DIRY = L6470_DIR_FWD_ID;
	else
		DIRY = L6470_DIR_REV_ID;

	/* Prepare the stepper driver to be ready to perform the Move command which produces
	 * a motion of abs(DY) microsteps along the DIRY direction*/
	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareMove(device, DIRY , abs(DY));

	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

	for (uint8_t device = L6470_ID(0); device <= L6470_ID(1); device++)
	{
		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(0, device, BUSY_ID) == 0);

	}

	int32_t X=AbsPos_2_Position(L6470_GetParam(L6470_ID(0), L6470_ABS_POS_ID));
	int32_t Y=AbsPos_2_Position(L6470_GetParam(L6470_ID(1), L6470_ABS_POS_ID));
	printf("Posx : %d[step]  Posy : %d[step] \n", X, Y);

	for (uint8_t device = L6470_ID(0); device <= L6470_ID(1); device++)
	{
		/* Prepare the stepper driver to be ready to perform the HardStop command
		 * which causes an immediate motor stop with infinite deceleration */
#ifdef HARDSTOP
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareHardStop(device);
#else
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareSoftStop(device);
#endif
	}

	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

	for (uint8_t device = L6470_ID(0); device <= L6470_ID(1); device++)
	{
		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(0, device, BUSY_ID) == 0);
	}
}

static void move2(int32_t DX, int32_t DY, double ratio){
	uint8_t device;
	eL6470_DirId_t DIRX, DIRY;
	float dx=(float) DX;
	float dy=(float) DY;
	float dr=sqrt(dx*dx+dy*dy);
	float ct=dx/dr;
	float st=dy/dr;

	float speed  = MotorParameterDataSingle->speed*ratio;

	device=L6470_ID(0);
	float speedx = speed*ct;
	if (speedx<0)
		DIRX = L6470_DIR_FWD_ID;
	else
		DIRX = L6470_DIR_REV_ID;

	uint32_t Speedx = Step_s_2_Speed(fabs(speedx));
	/* Prepare the stepper driver to be ready to perform the Run command which produces a motion at fixed speed.
	 * The speed value is in (([step/s] * 250e-9) / 2^-28) unit */
	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareSetParam(device, L6470_SPEED_ID, Speedx);

	device=L6470_ID(1);
	float speedy = speed*st;
	if (speedy<0)
		DIRY = L6470_DIR_FWD_ID;
	else
		DIRY = L6470_DIR_REV_ID;

	uint32_t Speedy = Step_s_2_Speed(fabs(speedy));
	/* Prepare the stepper driver to be ready to perform the Run command which produces a motion at fixed speed.
	 * The speed value is in (([step/s] * 250e-9) / 2^-28) unit */
	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareSetParam(device, L6470_SPEED_ID, Speedy);

	printf("Speedx : %f[step/s]  Speedy : %f[step/s]  ==> %lu %lu\n", device, speedx, speedy, Speedx, Speedy);

	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

	for (uint8_t device = L6470_ID(0); device <= L6470_ID(1); device++)
	{
		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(0, device, BUSY_ID) == 0);
	}

	device=L6470_ID(0);
	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareMove(device, DIRX , abs(DX));

	device=L6470_ID(1);
	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareMove(device, DIRY , abs(DY));

	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

	for (uint8_t device = L6470_ID(0); device <= L6470_ID(1); device++)
	{
		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(0, device, BUSY_ID) == 0);
	}

	for (uint8_t device = L6470_ID(0); device <= L6470_ID(1); device++)
	{
		/* Prepare the stepper driver to be ready to perform the HardStop command
		 * which causes an immediate motor stop with infinite deceleration */
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareHardStop(device);
	}

	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

	for (uint8_t device = L6470_ID(0); device <= L6470_ID(1); device++)
	{
		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(0, device, BUSY_ID) == 0);
	}
}

static uint32_t PCLK1TIM(void){ // F4 Example
	/* Get PCLK1 frequency */
	uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();

	/* Get PCLK1 prescaler */
	if((RCC->CFGR & RCC_CFGR_PPRE1) == 0){
		/* PCLK1 prescaler equal to 1 => TIMCLK = PCLK1 */
		return (pclk1);
	}
	else{
		/* PCLK1 prescaler different from 1 => TIMCLK = 2 * PCLK1 */
		return(2 * pclk1);
	}
}

static void TimerStart(double Ts){
	uint32_t PCLK1TIM_FREQ=PCLK1TIM();
	uint32_t PSC=TIM2->PSC;
	uint32_t FREQ = PCLK1TIM_FREQ/(PSC+1);
	uint32_t CNT = (uint32_t)(FREQ* Ts)-1;

	__HAL_TIM_SET_AUTORELOAD(&htim2, CNT);

	/*
	 * Setting the URS bit inside the TIMx->CR1 register: this will cause
that the UEV event is generated only when the counter reaches the overflow.
It is possible to configure the timer so that the ARR register is buffered,
by setting the TIM_CR1_ARPE bit in the TIMx->CR1 control register.
This will cause that the content of the shadow register is updated automatically
	 */
	TIM2->CR1 |= TIM_CR1_ARPE; //Enable preloading

	HAL_TIM_Base_Start_IT(&htim2);
	TIM2_AWAITING_UPDATE_EVENT=1;
}

static void TimerStop(void){
	HAL_TIM_Base_Stop_IT(&htim2);
}


/*
 * PSC=8399    PCLK1TIM_FREQ=84 MHz
 * FREQ = 84MHz/(PSC+1)= 10000 Hz --> T=1e-4 s
 * Ts = 10s
 * CNT = FREQ * Ts -1 = 1e5 -1 = 99 999
 */

static void wait(double Ts){
	uint32_t PCLK1TIM_FREQ=PCLK1TIM();
	uint32_t PSC=TIM2->PSC;
	uint32_t FREQ = PCLK1TIM_FREQ/(PSC+1);
	uint32_t CNT = (uint32_t)(FREQ* Ts)-1;

	__HAL_TIM_SET_AUTORELOAD(&htim2, CNT);
	HAL_TIM_Base_Start_IT(&htim2);
	TIM2_AWAITING_UPDATE_EVENT=1;
	while (TIM2_AWAITING_UPDATE_EVENT){ ; }
	HAL_TIM_Base_Stop_IT(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance==TIM2){
		TIM2_AWAITING_UPDATE_EVENT=0;
	}
}

static int32_t gcd(int32_t a, int32_t b){
	while (b != 0){
		int32_t r=a % b;
		a=b;
		b=r;
	}
	return a;
}

static void MoveBy(int32_t DX_target, int32_t DY_target){
	uint8_t device;
	eL6470_DirId_t DIR0, DIR1;
	int32_t DX, DY;

	int32_t DL0_target=DX_target-DY_target;
	int32_t DL1_target=DX_target+DY_target;

	int32_t res=1;
	int32_t GCD=gcd(DL0_target/res, DL1_target/res)*res;
	if (GCD == 0){
		return;
	}

	uint32_t NMaxStep = 2000;
	uint32_t NStep0 = abs(DL0_target/GCD);
	uint32_t NStep1 = abs(DL1_target/GCD);

	NStep0 = (uint32_t) (NStep0);
	NStep1 = (uint32_t) (NStep1);
	uint32_t NCHUNK  = abs((float) GCD);

#if DEBUG
	printf("0: NStep0 : %lu  NStep1 : %lu NCHUNK : %lu\n", NStep0, NStep1, NCHUNK);
#endif
	while ((NStep0<NMaxStep) && (NStep1<NMaxStep) && (NCHUNK>2)){
		NStep0 <<=1;
		NStep1 <<=1;
		NCHUNK >>=1;
	}

	while ((NStep0>NMaxStep) || (NStep1>NMaxStep)){
		NStep0 >>=1;
		NStep1 >>=1;
		NCHUNK <<=1;
	}
#if DEBUG
	printf("1: NStep0 : %lu  NStep1 : %lu NCHUNK : %lu\n", NStep0, NStep1, NCHUNK);
#endif

	float d2l0_target = (float) DL0_target / (float) NCHUNK;
	float d2l1_target = (float) DL1_target / (float) NCHUNK;

	int POS0=AbsPos_2_Position(BSP_L6470_GetParam(0, L6470_ID(0), L6470_ABS_POS_ID));
	int POS1=AbsPos_2_Position(BSP_L6470_GetParam(0, L6470_ID(1), L6470_ABS_POS_ID));

	device=L6470_ID(0);
	if (DL0_target > 0)
		DIR0 = L6470_DIR_FWD_ID;
	else
		DIR0 = L6470_DIR_REV_ID;

	device=L6470_ID(1);
	if (DL1_target > 0)
		DIR1 = L6470_DIR_FWD_ID;
	else
		DIR1 = L6470_DIR_REV_ID;

	float dl0_target=0;
	float dl1_target=0;

	for (uint32_t nchunk=0; nchunk<NCHUNK; nchunk++ ){
		/* Prepare the stepper driver to be ready to perform the Move command which produces
		 * a motion of NSTEEPX microsteps along the DIRX direction*/
		device=L6470_ID(0);
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareMove(device, DIR0 , NStep0);
		device=L6470_ID(1);
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareMove(device, DIR1 , NStep1);

		StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

		for (uint8_t device = L6470_ID(0); device <= L6470_ID(1); device++)
		{
			while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(0, device, BUSY_ID) == 0);
		}


		/* Prepare the stepper driver to be ready to perform the HardStop command
		 * which causes an immediate motor stop with infinite deceleration */
		device=L6470_ID(0);
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareSoftStop(device);
		device=L6470_ID(1);
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareSoftStop(device);

		StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

		for (uint8_t device = L6470_ID(0); device <= L6470_ID(1); device++)
		{
			while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(0, device, BUSY_ID) == 0);
		}

		int32_t DL0=AbsPos_2_Position(BSP_L6470_GetParam(0, L6470_ID(0), L6470_ABS_POS_ID))-POS0;
		int32_t DL1=AbsPos_2_Position(BSP_L6470_GetParam(0, L6470_ID(1), L6470_ABS_POS_ID))-POS1;
		dl0_target+= d2l0_target;
		dl1_target+= d2l1_target;

		DX=(+DL0+DL1)/2;
		DY=(-DL0+DL1)/2;

#if DEBUG
		printf("nchunk %ld  DX : %ld[step]  DY : %ld[step] NStep0 : %ld DL0 : %ld %f [step]  NStep1 : %ld DL1 : %ld %f[step]\n", nchunk, DX, DY,
				NStep0, DL0, dl0_target, NStep1, DL1, dl1_target);
#endif

		/* Prepare the stepper driver to be ready to perform the GoTo command which produces
		 *  a motion to ABS_POS absolute position through *** the shortest path *** */
		device=L6470_ID(0);
		int32_t L0 = (int32_t)(+dl0_target+POS0);
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareGoTo(device, Position_2_AbsPos(L0));

		/* Prepare the stepper driver to be ready to perform the GoTo command which produces
		 *  a motion to ABS_POS absolute position through *** the shortest path *** */
		device=L6470_ID(1);
		int32_t L1 = (int32_t)(+dl1_target+POS1);
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareGoTo(device, Position_2_AbsPos(L1));

		StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

		for (uint8_t device = L6470_ID(0); device <= L6470_ID(1); device++)
		{
			while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(0, device, BUSY_ID) == 0);
		}

		/* Prepare the stepper driver to be ready to perform the HardStop command
		 * which causes an immediate motor stop with infinite deceleration */
		device=L6470_ID(0);
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareSoftStop(device);
		device=L6470_ID(1);
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareSoftStop(device);

		StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

		for (uint8_t device = L6470_ID(0); device <= L6470_ID(1); device++)
		{
			while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(0, device, BUSY_ID) == 0);
		}
	}
}

/**
 * @}
 */ /* End of ExamplePrivateFunctions */

/**
 * @}
 */ /* End of Example */

/**
 * @}
 */ /* End of MicrosteppingMotor_Example */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
