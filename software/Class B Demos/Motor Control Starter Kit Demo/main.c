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
//  File:       main.c
//
// This program commutates a sensor less BLDC motor using closed PI loop.
// The circuit and operation is as depicted in AN1160.
//
//
// Includes:
//      int main(void)
//      void PreCommutationState(void)
//      void SpeedPILoopController(void)
//      void OpenLoopController(void)
//      void DelayNmSec(unsigned int N)
//      void StartMotor(void)
//      void StopMotor(void)
//      void InitMotor(void)
//
//                      Microchip Technology Inc
//
//
// The following files should be included in the MPLAB project:
//
//      main.c              -- Main source code file
//      interrupt.c         -- interrupt routines
//      init.c              -- initialization code
//      pi.c                -- PI controller inplementation
//      mcsk.h              -- project header file
//      pi.h                -- PI code header file
//
/*****************************************************************************/
//
//
//
//
/*****************************************************************************/
//
// Revision History
//
// 12/3/07  -- First Version Close Loop with BEMF sensing using the ADC
// 12/10/07 -- Adding comments and modifying global variables names
// 2/22/08  -- Spell Checking, Dead Variable Clean up, Ramp up bug
// 5/5/08   -- DMCI with real-time data monitor added
// 6/04/08  -- SW migrated to the dsPICDEM MCLV board
// 6/17/08  -- Commens addded, PI coefficients recalculated,
// 6/01/11  -- Updated for dsPIC33FJ16MC102 and Motor Control Starter Kit
//          -- Added CTMU Functionality
//          -- Updated defines for motor reference speed
// 6/08/11  -- reworked the Start-up procedure, PI controller, fault behaviour
//          -- added the state machne
//          -- split the project in multiple files
// 6/26/11  -- Added mTouch infrastructure
/*****************************************************************************/
#include "classb.h"
#include "mcsk.h"

/********************Setting Configuration Bits *********************************/
_CONFIG2(POSCMOD_NONE & LPOL_ON & OSCIOFNC_ON & IOL1WAY_OFF & FCKSM_CSECMD & FNOSC_FRCPLL & PWMPIN_ON & PWMLOCK_ON & IESO_OFF);

//_CONFIG1(       PLLKEN_ON & FWDTEN_OFF & ICS_PGD3 & HPOL_ON & GWRP_OFF & GCP_OFF);
_CONFIG1(0x3FFF & PLLKEN_ON & FWDTEN_OFF & ICS_PGD3 & HPOL_ON & GWRP_OFF & GCP_OFF);
/*****************************SYSTEM DEFINES***********************************/


/* Six-Step Commutation States*/
/*PHASE A is MI,  PHASE B is M2, PHASE C is M3 in the dsPICDEM MCLV board*/
/* State 0x2001 Connect the PHASE A to -Vbus & PHASE C to +Vbus */
/* State 0x2004 Connect the PHASE B to -Vbus & PHASE C to +Vbus */
/* State 0x0204 Connect the PHASE B to -Vbus & PHASE A to +Vbus */
/* State 0x0210 Connect the PHASE C to -Vbus & PHASE A to +Vbus */
/* State 0x0810 Connect the PHASE C to -Vbus & PHASE B to +Vbus */
/* State 0x0801 Connect the PHASE A to -Vbus & PHASE B to +Vbus */
const unsigned int PWM_STATE[]  =   {0x0000,0x2001,0x2004,0x0204,0x0210,0x0810,0x0801,0x0000};


/*AND & OR operators for masking the active BEMF signal*/
const unsigned int ADC_MASK[8]  =   {0x0000,0x0002,0x0001,0x0004,0x0002,0x0001,0x0004,0x0000};
const unsigned int ADC_XOR[8]   =   {0x0000,0x0000,0xFFFF,0x0000,0xFFFF,0x0000,0xFFFF,0x0000};

/*BEMF Majority Function Filter values*/
/*The Filter values are explained in AN1160 and they are listed in Table 7*/
const unsigned char ADC_BEMF_FILTER[64]=
                                    {0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x10,0x12,0x14,0x16,0x18,0x1A,0x1C,0x1E,
                                     0x20,0x22,0x24,0x26,0x28,0x2A,0x2C,0x2E,0x01,0x01,0x01,0x36,0x01,0x3A,0x3C,0x3E,
                                     0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x01,0x01,0x01,0x16,0x01,0x1A,0x1C,0x1E,
                                     0x01,0x01,0x01,0x26,0x01,0x2A,0x2C,0x2E,0x01,0x01,0x01,0x36,0x01,0x3A,0x3C,0x3E};

volatile SystemFlags Flags;
/*Boolean variables used to save the comparison between the phases and the neutral point*/
volatile ComparatorFlags Comparator;

/********************* Motor Control Variables *********************************/
volatile unsigned int PIticks;
volatile unsigned int stallCount;
volatile unsigned char ADCCommState;
volatile unsigned char adcBackEMFFilter;
unsigned int PhaseAdvance = 0;
volatile unsigned char BlankingCounter;

volatile unsigned int MotorNeutralVoltage;
volatile unsigned int MotorPhaseA;
volatile unsigned int MotorPhaseB;
volatile unsigned int MotorPhaseC;
volatile unsigned int ComparatorOutputs;
volatile unsigned int CurrentPWMDutyCycle;

unsigned int Timer2Value;
unsigned int Timer2Average;
unsigned int Timer1Value;

int state;                  //stateMachine variable

/********************* PID Varibles  *********************************/
int ReferenceSpeed;
#ifdef CLOSELOOPMODE
int DesiredSpeed, CurrentSpeed;
unsigned int SpeedControl_P = 200;      // The P term for the PI speed control loop
unsigned int SpeedControl_I = 100;      // The I term for the PI speed control loop

tPIParm PIDStructure;                   // PID Structure
#endif

///////////////////////////////////////////////////////////////////////// 
// CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB

#define RAM_BLOCK_LENGTH       128
#define RAM_START_ADDRESS      0x0880
#define RAM_END_ADDRESS        0x0C00

#define FLASH_BLOCK_LENGTH     128
#define FLASH_START_ADDRESS    0x0000
#define FLASH_END_ADDRESS      0x2C00

uint16_t   currentRAMAddress   = RAM_START_ADDRESS;
uint32_t   currentFlashAddress = FLASH_START_ADDRESS;
uint16_t   correctCRC = -1;
uint16_t   currentCRC = -1;

// CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB
///////////////////////////////////////////////////////////////////////// 

/******************************************************************************
* Function:     main(void)
*
* Output:       None
*
* Overview:     Main function used to init the ADC, PWM and TIMER2 modules.
*               It also inits the global variables used in the interrupts and
*               PI controller.
*               The main task executed here is to start and stop the motor
*               as well as setting the ramp-up initial parameters to
*               spin the motor
*
* Note:         None
*******************************************************************************/
int main(void)
{
    //Led ports
    TRISAbits.TRISA2 = 0;
    TRISBbits.TRISB8 = 0;
    TRISBbits.TRISB9 = 0;

    ///////////////////////////////////////////////////////////////////////// 
    // CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB

    // Check for startup march tests error.
    // These tests are run before main() function entry if CLASSB_MARCH_C_STARTUP,
    // CLASSB_MARCH_C_MINUS_STARTUP or/and CLASSB_MARCH_B_STARTUP flags are set
    // in classb_config.h file.  
    if(CLASSB_StartupMarchTestGetResult())
    {
       // ERROR IS DETECTED
       LED3 = 1;
       asm volatile ("bra $+0");
    }

    // Check for a watchdog timer timeout error.  
    if(RCONbits.WDTO == 1)
    {
       RCONbits.WDTO = 0;
       // ERROR IS DETECTED
       LED3 = 1;
       asm volatile ("bra $+0");
    }


    // Enable WDT
    RCONbits.SWDTEN = 1;

    // CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB
    //////////////////////////////////////////////////////////////////////////


    // Configure Oscillator to operate the device with internal FRC and PLL at 14.74Mhz
    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(0x01);

    while(OSCCONbits.COSC != 0b001);
    // Wait for PLL to lock
    while(OSCCONbits.LOCK != 1);

    /****************** Functions init *********************************/
    INTCON1bits.NSTDIS = 1;         //Disable nested interrupts
    InitMCPWM();                    //Configuring MC PWM module
    InitADC10();                    //Configuring ADC
    InitTMR2();                     //Configuring TIMER 3, used to measure speed
    InitTMR1();                     //Configuring TIMER 1, used for the commutation delay

    #ifdef DEBUG_APP
    TRISBbits.TRISB9 = 1;           //in debug mode this pin is used for triggering the CTMU events
                                    //the D7 LED cannot be used in Debug mode
    #endif

    //Push Buttons ports
    TRISAbits.TRISA3 = 1;

    //Turn LED's OFF
    LED1 = 0;Nop();                 //a Nop() is needed between these instructions because of read-modify-write operations
    LED2 = 0;Nop();

    //itit state machine variables
    state = STOP;

    int buttonPressed = 0;
    int buttonCounter = 0;

    /****************** Infinite Loop *********************************/
    while(1)
    {

        ///////////////////////////////////////////////////////////////////////// 
        // CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB

        // CPU Program Counter test.
        // Should be used with a watchdog timer.

        // Check program counter status everywhere using CLASSB_CPUPCTestGetResult()
        // especially before critical code execution (such as motor on/off, power enable and so on).
        if(CLASSB_CPUPCTestGetResult())
        {
            // ERROR IS DETECTED
            LED3 = 1;
            asm volatile ("bra $+0");
        }
        CLASSB_CPUPCTest();
        // Clear watchdog.
        ClrWdt();

        // CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB
        ///////////////////////////////////////////////////////////////////////// 


        ///////////////////////////////////////////////////////////////////////// 
        // CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB

        // CPU Registers test.
        if(CLASSB_CPURegistersTest())
        {
            // ERROR IS DETECTED
            LED3 = 1;
            asm volatile ("bra $+0");
		}

        // CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB
        ///////////////////////////////////////////////////////////////////////// 


        ///////////////////////////////////////////////////////////////////////// 
        // CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB

        // 1KBytes RAM memory test (block by block)

        // Checker Board test
        if(CLASSB_RAMCheckerboardTest(currentRAMAddress, // start address (must be aligned by word (2 bytes))
                                      RAM_BLOCK_LENGTH)) // length in bytes (must be aligned by 4 bytes)
        {
            // ERROR IS DETECTED
            LED3 = 1;
            asm volatile ("bra $+0");

        }
        // Move to the next memory block.
        currentRAMAddress += RAM_BLOCK_LENGTH;

        // If was last RAM block, move to the RAM start.
        if(currentRAMAddress >= RAM_END_ADDRESS)
        {
            currentRAMAddress = RAM_START_ADDRESS;
        }

        // CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB
        ///////////////////////////////////////////////////////////////////////// 


        ///////////////////////////////////////////////////////////////////////// 
        // CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB

        // 16KBytes Flash memory test (block by block)

        // Calcultate CRC of the flash memory block.
        currentCRC = CLASSB_CRCFlashTest(currentFlashAddress, // start address (must be aligned by word (2 bytes))
                                  FLASH_BLOCK_LENGTH,         // length in program counter units (addressable bytes, must be even)
                                  currentCRC);                // initial/previous CRC to continue calculations  


        // Move to the next memory block.
        currentFlashAddress += FLASH_BLOCK_LENGTH;

        // If it was last flash block, move to the flash start.
        // Compare CRC with precalculated value.
        if(currentFlashAddress >= FLASH_END_ADDRESS)
        {
            if(correctCRC != -1)
            { 
                if(currentCRC != correctCRC)
                {
                    LED3 = 1;
                    asm volatile ("bra $-2");
                }
            }else{
                // Assume that the first cycle CRC is correct.
                correctCRC = currentCRC; 
            }
            LED1 ^= 1;
            currentFlashAddress = FLASH_START_ADDRESS; // Move to the flash start.
            currentCRC = -1; // Set initial value for the next cycle.
        }

        // CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB
        //////////////////////////////////////////////////////////////////////////


        //check if the button was pressed

        if(!S1)                                     //button is pressed
        {
            //keep counting if button pressed and debounce value not reached
            if(++buttonCounter > BUTTON_FILTER)
            {
                buttonCounter = BUTTON_FILTER+1;    //limit the counter to avoid overflows
                if( buttonPressed == 0) //button pressed and debounced
                {
                    buttonPressed = 1;              //set button presseed flag
                }
            }
        }
        else                                        //button is not pressed
        {
            buttonCounter = 0;                      //clear counter and flag
            buttonPressed = 0;
        }

        //StateMachine
        switch(state)
        {
            case STOP: //motor is stopped
                if((buttonPressed == 1)) //a button was pressed, start the motor
                {
                    InitMotor();                //reinitialize all motor data
                    state = STARTUP;            //set state and motor flag to startup
                    Flags.Startup = 1;
                    buttonPressed = 2;          //set the flag to 2 to avoid multiple actions per one button push
                    LED1 = 0;
                    LED2 = 0;
                    LED3 = 0;

                }
                break;

            case STARTUP:                       //motor is running in forced commutation waiting for a valid BEMF zero crossing event to transition to
                if(Flags.Startup == 1)          //if the motor has not started, keep spinning by forced commutaion
                {                               //the Startup flag is cleared when a valid Zero crossing event is detected on one BEMF signal
                    StartMotor();               //forced commutation function used to stop the motor
                }
                else                            //BEMF zero crossing detected -> ending the start-up phase
                {
                    state = RUN;
                    ReferenceSpeed = MAX_MOTOR_SPEED_REF;
                }

                if(buttonPressed == 1)          //the button was pressed
                {
                    StopMotor();                //stop the motor, disable PWM outputs and timers
                    state = STOP;
                    buttonPressed = 2;          //set the flag to 2 to avoid multiple actions per one button push
                }

                break;

            case RUN:                           //motor is running in closed loop or open loop
                if(buttonPressed == 1)          //the button was pressed
                {
                    StopMotor();                //stop the motor, disable PWM outputs and timers
                    state = STOP;
                    buttonPressed = 2;          //set the flag to 2 to avoid multiple actions per one button push
                }
                break;

            case FAULT:                         //overcurrent or rotor stall have occured
                if(Flags.RunMotor == 1)         //if the motor was running, restart it
                {

                    StopMotor();                //stop the motor, disable PWM outputs and timers
                    InitMotor();                //reinitialize all motor data
                    Flags.Startup = 1;          //set state and motor flag to startup
                    state = STARTUP;
                }
                else
                {
                    state = STOP;               //if the motor was stopped return to STOP state
                }

                break;

            default:
                state = STOP;              //if the state variable has an invalid value set the FAULT state
                break;

        }//end switch(state)

    }//end while(1)

 return 0;
}//end of main function


/******************************************************************************
* Function:     PreCommutationState(void)
*
* Output:       None
*
* Overview:     This function measures the 60 and 30 electrical degrees
*               using the TIMER2. The 60 electrical degrees is proportional
*               to the elpased time between zero-crossing events.
*               The zero-crossing events occur 30 electrical degrees in advace
*               of the commutation point. Hence a delay proportional to the 30
*               electrical degrees is added using the TIMER1
*               The function is called in the PWM interrupt
*
* Note:         None
*******************************************************************************/
void PreCommutationState(void)
{

    // Calculate the time proportional to the 60 electrical degrees
    T2CONbits.TON = 0;  // Stop TIMER2
    Timer2Average = ((2*Timer2Average + Timer2Value + TMR2)>>2);
    Timer2Value = TMR2;
    TMR2 = 0;
    T2CONbits.TON = 1;  // Start TIMER2

    //Calculate the delay in TIMER1 counts proportional to the Phase Adv angle
    PhaseAdvance = ((Timer2Average*PHASE_ADVANCE_DEGREES)/60);

    // Calculate the time proportional to the 30 electrical degrees
    // Load the TIMER1 with  the TIMER1 counts porportional to 30 deg   minus the PHASE ADV angle delay
    Timer1Value = (((Timer2Average)>>1)-PhaseAdvance);
    if(Timer1Value>1)
        PR1 = Timer1Value;
    else
        PR1 = Timer1Value = 1;

    // Start TIMER1
    T1CONbits.TON = 1;

    // Change The Six-Step Commutation Sector
    adcBackEMFFilter=0;
    if (++ADCCommState>6)
        ADCCommState=1;

    ///////////////////////////////////////////////////////////////////////// 
    // CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB

    // CPU Program Counter test.

    // Check program counter status everywhere using CLASSB_CPUPCTestGetResult()
    // especially before critical code execution (such as motor on/off, power enable and so on).
    if(CLASSB_CPUPCTestGetResult())
    {
        // ERROR IS DETECTED
        LED3 = 1;
        asm volatile ("bra $+0");
    }
    // CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB  CLASSB
    ///////////////////////////////////////////////////////////////////////// 

}

/******************************************************************************
* Function:     SpeedPILoopController(void)
*
* Output:       None
*
* Overview:     When the PI button is ON on the DMCI window
*               the motor operates in close loop mode. The Kp and Ki
*               parameters were determined using the HURST MOTOR shipped with
*               the MCLV board. These values should be modified according to
*               the motorand load characteristics.
*
* Note:         None
*******************************************************************************/
#ifdef CLOSELOOPMODE
void SpeedPILoopController(void)
{
    //For the HURST motor the Timer2Avg values
    //TIMER2 counts = 142 AT 100% dutycycle (P1DC1 = 734)
    //TIMER2 counts = 2139 AT 4.9% dutycycle (P1DC1 = 46)
    DesiredSpeed = ReferenceSpeed;

    // Normalizing TIMER2 counts to electrical RPS expressed in PWM counts
    // Timer 2 Counts are converted to PWM counts
    // and then multipied by the number of sector
    // required to complete 1 electrical RPS
    CurrentSpeed = (int)(__builtin_divud(SPEEDMULT,Timer2Average));

    //PID controller
    PIDStructure.qInRef = (int)DesiredSpeed;
    PIDStructure.qInMeas = (int)CurrentSpeed;
    CalcPI(&PIDStructure);


    CurrentPWMDutyCycle =  PIDStructure.qOut;      //set PID output
    P1DC1 = CurrentPWMDutyCycle;
    P1DC2 = CurrentPWMDutyCycle;
    P1DC3 = CurrentPWMDutyCycle;

}

/******************************************************************************
* Function:     OpenLoopController(void)
*
* Output:       None
*
* Overview:     When the PI button is OFF on the DMCI window
*               the motor operates in open loop mode.
*
* Note:         None
*******************************************************************************/
#else
void OpenLoopController(void)
{

unsigned int DesiredPWMDutyCycle;

    //Scale Reference speed to PWM duty cycle; 100% duty cycle corresponds to MAX_MOTOR_SPEED_REF
    DesiredPWMDutyCycle = (int)(((long)ReferenceSpeed * MAX_DUTY_CYCLE)/MAX_MOTOR_SPEED_REF);

    //Update the duty cycle according to the POT value, a POT follower is implemented here
    if(CurrentPWMDutyCycle != DesiredPWMDutyCycle)
        {
        if(CurrentPWMDutyCycle < DesiredPWMDutyCycle)
            CurrentPWMDutyCycle++;
        if(CurrentPWMDutyCycle > DesiredPWMDutyCycle)
            CurrentPWMDutyCycle--;
        }
    // Max and Min PWM duty cycle limits
    if (CurrentPWMDutyCycle < MIN_DUTY_CYCLE)
        CurrentPWMDutyCycle = MIN_DUTY_CYCLE;
    if (CurrentPWMDutyCycle > MAX_DUTY_CYCLE)
        CurrentPWMDutyCycle = MAX_DUTY_CYCLE;
    //Assigning new duty cycles to the PWM channels
    P1DC1 = CurrentPWMDutyCycle;
    P1DC2 = CurrentPWMDutyCycle;
    P1DC3 = CurrentPWMDutyCycle;

}
#endif


/******************************************************************************
* Function:     DelayNmSec(unsigned int N)
*
* Output:       None
*
* Overview:     Delay function used for the motor start-up sequence
*
* Note:         None
*******************************************************************************/
void DelayNmSec(unsigned int N)
{
    while(N--)
    {
        DELAY_1MS;
    }

}


/******************************************************************************
* Function:     StartMotor(void)
*
* Output:       None
*
* Overview:     This function implements force commutation startup of the motor
*
*
* Note:         None
*******************************************************************************/
void StartMotor(void)
{
    // Change The Six-Step Commutation Sector
    if(++ADCCommState >6)
    ADCCommState = 1;
    P1OVDCON=PWM_STATE[ADCCommState];

    //clear the BEMF filter
    adcBackEMFFilter=0;

    //Delay between two commutations
    DelayNmSec(STARTUP_MSEC);

}


/******************************************************************************
* Function:     StopMotor(void)
*
* Output:       None
*
* Overview:     This function stops the motor by driving all PWM pins LOW; it also
*               stops the timers and turns the LED indicators OFF
*
*
* Note:         None
*******************************************************************************/
void StopMotor(void)
{

    // disable PWM outputs
    P1OVDCON = 0x0000;          // override PWM pins low
    Flags.RunMotor = 0;         // reset run flag

    T1CONbits.TON = 0;  // Stop TIMER1
    T2CONbits.TON = 0;  // Stop TIMER2
    TMR2 = 0;

    //Turn LED's OFF
    LED1 = 0;Nop();                 //a Nop() is needed between these instructions because of read-modify-write operations
    LED2 = 0;Nop();
    LED3 = 0;

}

/***************************************************************************
* Function:     InitMotor(void)
*
* Output:       None
*
* Overview:     This function initializes the PI controller, Timer2 average
*               and the initial motor reference speed and duty cycle
*
* Note:         None
****************************************************************************/
void InitMotor(void)
{
    //reset the stall detection counter
    stallCount = 0;


#ifdef CLOSELOOPMODE
    //Init PI controller variables
    DesiredSpeed = ReferenceSpeed;
    CurrentSpeed = MIN_MOTOR_SPEED_REF;
    // load the PID coeffecients
    InitPI(&PIDStructure,SpeedControl_P,SpeedControl_I,0,MAX_DUTY_CYCLE,MIN_DUTY_CYCLE,MIN_DUTY_CYCLE);
#endif

    ReferenceSpeed =  MIN_MOTOR_SPEED_REF;  //Set the minimum reference speed

    CurrentPWMDutyCycle = MIN_DUTY_CYCLE;   //Init PWM duty cycle value to minimum duty allowed
    P1DC1=P1DC2=P1DC3=MIN_DUTY_CYCLE;

    //initialize TMR2 and TMR2 average with the value corresponding to the minimum motor speed
    Timer2Average = Timer2Value = TMR2 = (int)(SPEEDMULT/MIN_MOTOR_SPEED_REF);

    Flags.RunMotor = 1;                     // turn the motor ON

}


