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
//  File:       interrupt.c
//  This file implements the interrupt service routines for
//              ADC,PWM, FAULTA and TIMER1
//
//  Includes:
//      void __attribute__((__interrupt__,auto_psv)) _ADC1Interrupt(void)
//      void __attribute__((__interrupt__,auto_psv)) _MPWM1Interrupt(void)
//      void __attribute__((__interrupt__,auto_psv)) _FLTA1Interrupt(void)
//      void __attribute__((__interrupt__,auto_psv)) _T1Interrupt(void)
//
/*****************************************************************************/

#include "mcsk.h"

/******************************************************************************
* Function:     _ADCInterrupt(void)
*
* Output:       None
*
* Overview:     ADC interrupt used to measure the BEMF signals, reconstruct
*               the Motor Virtual Neutral Point and compare the BEMF signals
*               against the neutral point reference to detect the zero-crossing
*               event
*               The 2-Channel Capacitve Slider and LED indicators are implemented here
*
* Note:         None
*******************************************************************************/

void __attribute__((__interrupt__,auto_psv)) _ADC1Interrupt(void)
{
    MotorPhaseA = ADC1BUF1;     //ADC CH1 holds the Phase A value
    MotorPhaseB = ADC1BUF2;     //ADC CH2 holds the Phase B value
    MotorPhaseC = ADC1BUF3;     //ADC CH3 holds the Phase C value
    //Reconstrucs Voltage at the  Motor Neutral Point
    MotorNeutralVoltage = (MotorPhaseA + MotorPhaseB + MotorPhaseC)/3;

    /********************* ADC SAMPLING & BMEF signals comparison ****************/
    if(BlankingCounter > BLANKING_COUNT){
        ComparatorOutputs = 0;                      // Precondition all comparator bits as zeros
        if(MotorPhaseA > MotorNeutralVoltage)
            ComparatorOutputs += 1;                 // Set bit 0 when Phase C is higher than Neutural
        if(MotorPhaseB > MotorNeutralVoltage)
            ComparatorOutputs += 2;                 // Set bit 1 when Phase C is higher than Neutural
        if(MotorPhaseC > MotorNeutralVoltage)
            ComparatorOutputs += 4;                 // Set bit 2 when Phase C is higher than Neutral
    }
    //clear the interrupt flag

    IFS0bits.AD1IF = 0;

}


/******************************************************************************
* Function:     _PWMInterrupt(void)
*
* Output:       None
*
* Overview:     PWM reload interrupt used to filter the BEMF signals using the
*               Majority detection filter to detect a valid zero-crossing event
*               if a valid zero-crossing event was detected then PreCommutationState.
*               This function also includes the start-up sequence for detecting
*               the initial rotor position
*
* Note:         None
*******************************************************************************/
void __attribute__((__interrupt__,auto_psv)) _MPWM1Interrupt(void)
{

    IFS3bits.PWM1IF = 0;
    //no action is performed if the motor is stopped
    if(Flags.RunMotor == 1)
    {

        //Sets the ADC sampling point according to the PWM duty cycle
        //Please refer to AN1160 for details regarding the sampling timing of the BEMF signals
        if(CurrentPWMDutyCycle>160)
            P1SECMPbits.SEVTCMP = CurrentPWMDutyCycle>>2;
        else if(CurrentPWMDutyCycle>76)
            P1SECMPbits.SEVTCMP = CurrentPWMDutyCycle>>4;
        else
            P1SECMPbits.SEVTCMP = 0;


        // Masking the BEMF signals according to the SECTOR in order to determine the ACTIVE BEMF signal
        // XOR operator helps to determine the direction of the upcoming zero-crossing slope
        BlankingCounter++;
        if(BlankingCounter > BLANKING_COUNT)
        {
            if((ComparatorOutputs^ADC_XOR[ADCCommState])& ADC_MASK[ADCCommState])
            {
                adcBackEMFFilter|=0x01;
            }

            //Majority detection filter
            adcBackEMFFilter = ADC_BEMF_FILTER[adcBackEMFFilter];
            if (adcBackEMFFilter&0b00000001)
            {
                //When a valid BEMF zero crossing event has been detected, disable the motor start-up sequence
                Flags.Startup = 0;

                //clear the stall counter whenever the BEMF signal is detected
                stallCount = 0;

                //measure the commutation timing and set the new sector
                PreCommutationState();
            }
            else
            {
                //if a BEMF zero crossing was not detected increment the stall counter
                ++stallCount;
            }
        }

        //rotor stall detection
        if (stallCount > BEMF_STALL_LIMIT && state == RUN)
        {
            state = FAULT;      //go to FAULT state and restart the motor without pushing the button
            stallCount = 0;     //clear the stall counter
        }

        //Call the PI speed controller at a fixed frequency, which is (PI_TICKS*50us)
        if(++PIticks >= PI_TICKS)
        {
        #ifdef CLOSELOOPMODE
            SpeedPILoopController();
        #else
            OpenLoopController();
        #endif
            PIticks = 0;
        }

    }

}

/******************************************************************************
* Function:     _FLTA1Interrupt(void)
*
* Output:       None
*
* Overview:     In case of an overcurrent, the motor is stopped and restarted
*
* Note:         None
*******************************************************************************/
void __attribute__((__interrupt__,auto_psv)) _FLTA1Interrupt(void)
{

    P1OVDCON = 0x0000;              //set all PWM pins to LOW

    state = FAULT;                  //set the current state to FAULT to allow the motor to restart

    IFS3bits.FLTA1IF = 0;           //clear interrupt flag

}

/******************************************************************************
* Function:      _T1Interrupt(void)
*
* Output:       None
*
* Overview:     Here is where the motor commutation occurs,
*               Timer1 ISR is utilized as the commutation delay used to
*               commutate the motor windings at the right time
*
* Note:         None
*******************************************************************************/
void __attribute__((__interrupt__,auto_psv)) _T1Interrupt(void)
{
    //set the commutation sector
    P1OVDCON=PWM_STATE[ADCCommState];

    BlankingCounter = 0;

    IFS0bits.T1IF = 0;      // Clear Timer 1 Interrupt Flag
    T1CONbits.TON = 0;      // Stop TIMER1
    TMR1 = 0;

}
