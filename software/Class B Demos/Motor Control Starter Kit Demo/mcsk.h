#ifndef MCSK_H
#define MCSK_H

/******************************************************************************/
//
//                           Software License Agreement
//
// CODE OWNERSHIP AND DISCLAIMER OF LIABILITY

// Microchip Technology Incorporated ("Microchip") retains
// all ownership and intellectual property rights in the code
// accompanying this message and in all derivatives hereto.
// You may use this code, and any derivatives created by any
// person or entity by or on your behalf, exclusively with
// Microchip’s proprietary products.  Your acceptance and/or
// use of this code constitutes agreement to the terms and
// conditions of this notice.

// CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING,
// BUT NOT LIMITED TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT,
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO
// THIS CODE, ITS INTERACTION WITH MICROCHIP’S PRODUCTS, COMBINATION
// WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

// YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE
// LIABLE, WHETHER IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE
// OR BREACH OF STATUTORY DUTY), STRICT LIABILITY, INDEMNITY, CONTRIBUTION,
// OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, PUNITIVE, EXEMPLARY, INCIDENTAL
// OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF ANY KIND WHATSOEVER
// RELATED TO THE CODE, HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED
// OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
// ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
// RELATED TO THIS CODE, SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO
// MICROCHIP SPECIFICALLY TO HAVE THIS CODE DEVELOPED.

// You agree that you are solely responsible for testing the code and
// determining its suitability.  Microchip has no obligation to modify,
// test, certify, or support the code.

/*****************************************************************************/
//  File:       mcsk.h
/*****************************************************************************/

#include <xc.h>
#include "pi.h"

#define DEBUG_APP // Use LED3 for diagnostic outputs
#undef  DEBUG_APP

/*FREQUENCY SYSTEM DEFINES*/
#define FCY         14740000                        //internal FRC with x4 PLL -> 7.37MHz*4/2 = 14.740 Mhz
#define MILLISEC    FCY/14740                       //1 mSec delay constant
#define FPWM        20000                           //20KHz PWM Freq
#define DELAY_1MS   {asm("repeat #14740");Nop();}   //1ms delay

/******************************************************************************
* CLOSED/OPEN LOOP SPEED CONTROLLER DEFINITION
* If defined then the compiler will add the PID controller for speed
*******************************************************************************/
#define CLOSELOOPMODE

/********************* Motor Definitions **************************/

#define MAX_RPM                 1900    // motor RPM at 100% duty cycle
#define POLEPAIRS               6       // Number of pole pairs of the motor

/********************* Startup Definitions************************/

#define STARTUP_MSEC            10      //time in miliseconds between two commutation steps (1/6 of each electrical rotation)
                                        //sets the starting motor speed in forced commutation mode; increase the value for a lower speed
#define MIN_DUTY_CYCLE          150     //sets the minimum duty cycle at which the motor starts in forced commutation mode

/********************* Tuning Definitions************************/

#define PHASE_ADVANCE_DEGREES   20      // Phase advance angles to get the best motor performance
#define BEMF_STALL_LIMIT        5000    // If no BEMF signal is detected for (BEMF_STALL_LIMIT*BLANKING_COUNT * 50us) then it is assumed the rotor is stalled
#define BLANKING_COUNT          5       // Blanking count expressed in PWM periods used to avoid false zero-crossing detection after commutating motor

//Scaling values and limits for the motor speed reference
#define MAX_MOTOR_SPEED_REF     4000    // corresponds to MAX_RPM
#define MIN_MOTOR_SPEED_REF     1000    // decrease or increase this value to set the minimum motor speed
                                        // The minimum motor speed in closed loop is MAX_RPM*MIN_MOTOR_SPEED_REF/MAX_MOTOR_SPEED_REF


/*******************  Derived Definitions  - Do not change*******************/

#define PI_TICKS                    80                          // Speed Controller frequency ->  80 PWM periods equals 4 ms
#define MAX_DUTY_CYCLE              (int)(((FCY/FPWM)/2-1)*2)   // 100% duty cycle P1DC1=P1DC2=P1DC3 = 734

//Timer2 measures the motor speed by measuring the time the rotor takes to make a 60 degree electrical rotation angle
//Timer2 minimum value is: 1/(MAX_RPM/60)/POLEPAIRS*FCY/Timer2Prescaler/(360/60)
#define TMR2_MIN                    (double)60/MAX_RPM/POLEPAIRS*FCY/64/6   //Timer2 prescaler is 64

// CONVERSION SPEED FACTOR - SPEEDMULT
#define SPEEDMULT                   (long)(MAX_MOTOR_SPEED_REF * TMR2_MIN)  //Foactor used to scale the Desired speed to the actual motor speed


/********************* User interface defines ****************************/
//Push button
#define S1              PORTAbits.RA3   //Defines for the Push Buttons status

//Define LED ports
#define LED1            PORTAbits.RA2   //D5 LED
#define LED2            PORTBbits.RB8   //D6 LED
#define LED3            PORTBbits.RB9   //D7 LED

//button debounce filter limit
#define BUTTON_FILTER       100

//motor speed and reference defines
#define BLINK_FCY           2           // sdjusts the LED blinking frequency in SetLEDSpeed()
#define SLIDER_RES         20           // adjusts the slider resolution; decrease the value for more resolution

//CTMU Slider defines
#define SLIDER_UPDATE_FREQ   500        //slider update frequency (SLIDER_UPDATE_FREQ*50us)
#define SLIDER_MIN_INCREMENT 3         //minimum incremnent allowed to the ReferenceSpeed

/********************* Control defines ****************************/

//state machine defines
#define STOP        0
#define STARTUP     1
#define RUN         2
#define FAULT       3


/********************* End of defines ****************************/


/* Six-Step Commutation States*/
/*PHASE A is MI,  PHASE B is M2, PHASE C is M3 in the dsPICDEM MCLV board*/
/* State 0x2001 Connect the PHASE A to -Vbus & PHASE C to +Vbus */
/* State 0x2004 Connect the PHASE B to -Vbus & PHASE C to +Vbus */
/* State 0x0204 Connect the PHASE B to -Vbus & PHASE A to +Vbus */
/* State 0x0210 Connect the PHASE C to -Vbus & PHASE A to +Vbus */
/* State 0x0810 Connect the PHASE C to -Vbus & PHASE B to +Vbus */
/* State 0x0801 Connect the PHASE A to -Vbus & PHASE B to +Vbus */
extern const unsigned int PWM_STATE[];


/*AND & OR operators for masking the active BEMF signal*/
extern const unsigned int ADC_MASK[8];
extern const unsigned int ADC_XOR[8];

/*BEMF Majority Function Filter values*/
extern const unsigned char ADC_BEMF_FILTER[64];

/*Application Flags to indicate the motor status*/

/*Application Flags to indicate the motor status*/
typedef struct {
            unsigned RunMotor :       1;
            unsigned Startup        : 1;
            unsigned unused :         14;
        }SystemFlags;

extern volatile SystemFlags Flags;
/**/
/*Boolean variables used to save the comparison between the phases and the neutral point*/

typedef struct {

            unsigned PhaseAOutput :     1;
            unsigned PhaseBOutput :     1;
            unsigned PhaseCOutput :     1;
            unsigned unused       :     13;

        }ComparatorFlags;

extern volatile ComparatorFlags Comparator;
/**/
/********************* Motor Control Varaibles *********************************/
extern volatile unsigned int PIticks;
extern volatile  unsigned int stallCount;
extern volatile  unsigned char ADCCommState;
extern volatile  unsigned char adcBackEMFFilter;
extern unsigned int PhaseAdvance;
extern volatile  unsigned char BlankingCounter;

extern volatile unsigned int MotorNeutralVoltage;
extern volatile unsigned int MotorPhaseA;
extern volatile unsigned int MotorPhaseB;
extern volatile unsigned int MotorPhaseC;
extern volatile unsigned int ComparatorOutputs;
extern volatile unsigned int CurrentPWMDutyCycle;

extern unsigned int Timer2Value;
extern unsigned int Timer2Average;
extern unsigned int Timer1Value;

extern int state;               //stateMachine variable

/********************* PID Varibles  *********************************/
extern int ReferenceSpeed;
#ifdef CLOSELOOPMODE
extern int DesiredSpeed, CurrentSpeed;
extern unsigned int SpeedControl_P;     // The P term for the PI speed control loop
extern unsigned int SpeedControl_I;     // The I term for the PI speed control loop

extern tPIParm PIDStructure;                    // PID Structure
#endif


void InitMCPWM(void);
void InitADC10(void);
void InitTMR2(void);
void InitTMR1(void);
void PreCommutationState(void);
void SpeedPILoopController(void);
void OpenLoopController(void);
void DelayNmSec(unsigned int N);
void StartMotor(void);
void StopMotor(void);
void InitMotor(void);


#endif
