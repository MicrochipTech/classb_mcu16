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
//  File:       init.c
//  This file implements the initialization routines for
//              ADC,PWM, TIMER1, TIMER2, IC and OC
//
//  Includes:
//      void InitADC10(void)
//      void InitMCPWM(void)
//      void InitTMR2(void)
//      void InitTMR1(void)
//
/*****************************************************************************/

#include "mcsk.h"

/******************************************************************************
* Function:     InitADC10(void)
*
* Output:       None
*
* Overview:     Initializes the ADC module to operate in simultaneous mode
*               sampling terminals AN0,AN1, AN2, AN3 using MUX A. The ADC channels are
*               assigned as follows:
*               CH0->AN0/AN1 Capacitive Slider Chammels
*               CH1->AN3 BEMF PHASE A
*               CH2->AN4 BEMF PHASE B
*               CH3->AN5 BEMF PHASE C
*               ADC is sync with the PWM. ADC is conversion is triggered
*               every time a PWM reload event occurs. Tadc = 4*TCY = 271.37 nSec.
*               ADC resulting samples are formatted as unsigned 10-bits
*               Right-justified
*
* Note:         None
*******************************************************************************/
void InitADC10(void)
{

    AD1PCFGL = 0x0; //Port pin multiplexed with AN0-AN8 in Analog mode

    AD1CON1 = 0x006C;   //ADC is off
                        //Continue module operation in Idle mode
                        //10-bit, 4-channel ADC operation
                        //Data Output Format bits Integer (0000 00dd dddd dddd)
                        //011 = Motor Control PWM interval ends sampling and starts conversion
                        //Samples CH0, CH1, CH2, CH3 simultaneously when CHPS<1:0> = 1x
                        //Sampling begins immediately after last conversion SAMP bit is auto-set.

    AD1CHS123 = 0x0001; //MUX B CH1, CH2, CH3 negative input is VREF-
                        //MUX B CH1 positive input is AN0, CH2 positive input is AN1, CH3 positive input is AN2
                        //MUX A CH1, CH2, CH3 negative input is VREF-
                        //MUX A CH1 positive input is AN3, CH2 positive input is AN4, CH3 positive input is AN5


    AD1CHS0 = 0x0000;   //MUX B Channel 0 negative input is VREF-
                        //MUX B Channel 0 positive input is AN0
                        //MUX A Channel 0 negative input is VREF-
                        //MUX A Channel 0 positive input is AN0

    AD1CSSL = 0x0000;   //Skip all ANx channels for input scan

    AD1CON3 = 0x0003;   //ADC Clock derived from system clock
                        //Autosample time time bits = 0 TAD since PWM is controlling sampling time
                        //TAD = 4*TCY, TAD = 271.37 nSec

    AD1CON2 = 0x0300;   //ADREF+ = AVDD ADREF- = AVSS
                        //Do not scan inputs
                        //1x = Converts CH0, CH1, CH2 and CH3
                        //A/D is currently filling buffer 0x0-0x7
                        //Interrupts at the completion of conversion for each sample/convert sequence
                        //Always starts filling buffer from the beginning
                        //Always uses channel input selects for Sample A

    AD1CON1bits.DONE = 0;   //Making sure that there is any conversion in progress
    IPC3bits.AD1IP = 5;     //Assigning ADC ISR priority
    IFS0bits.AD1IF = 0;     //Clearing the ADC Interrupt Flag
    IEC0bits.AD1IE = 1;     //Enabling the ADC conversion complete interrupt
    AD1CON1bits.ADON = 1;   //Enabling the ADC module

}


/******************************************************************************
* Function:     InitMCPWM(void)
*
* Output:       None
*
* Overview:     Initializes the PWM module to operate in center-aligned mode
*               at 20KHz. PWM terminals are configured in independent mode.
*               PWM time base is 67.8 nSec.
*               PDCx value range is 0-1464 for 0%-100% duty cycle
*               ADC reload time is variable according to the PWM duty cycle
*
*
* Note:         None
*******************************************************************************/
void InitMCPWM(void)
{

    P1TPER =((FCY/FPWM)/2 - 1); //FCY  29491200...FRC w/PLL x16
                                //FPWM 20KHz PWM Freq
                                // MAX_DUTY_CYCLE = 1469
                                // 50% duty cycle = 734


    P1TCONbits.PTSIDL = 1;      // PWM time base halted in CPU IDLE mode
    P1TCONbits.PTOPS = 0;       // PWM time base 1:1 postscale
    P1TCONbits.PTCKPS = 0;      // PWM time base 1:1 prescale
    P1TCONbits.PTMOD = 2;       // Center Aligned with single interrupt mode per PWM period

    P1OVDCON = 0x0000;      // set all PWM outputs to LOW using OVD

    // enable PWM outputs
    __builtin_write_PWMSFR(&PWM1CON1, 0x0777, &PWM1KEY);



    P1SECMPbits.SEVTDIR = 0;    // trigger ADC when PWM counter is in upwards dir
    P1SECMPbits.SEVTCMP = 0;    // generates a trigger event for the ADC
                                // when PWM time base is counting upwards
                                // just before reaching the PTPER value
                                // causing a sampling in the middle of the
                                // pulse

    PWM1CON2 = 0x0000;          // 1:1 postscale values
                                // Updates to the active PxDCy registers
                                // are sync to the PWM time base
                                // Output overrides via the PxOVDCON register occur
                                // on the next TCY boundary
                                // Updates from duty cycle and period buffer registers
                                // are enabled

    IPC14bits.PWM1IP = 4;       // PWM Interrupt Priority 4
    IFS3bits.PWM1IF=0;          // Clearing the PWM Interrupt Flag
    IEC3bits.PWM1IE=1;          // Enabling the PWM interrupt


    //faultA enabled, FaultB disabled
    __builtin_write_PWMSFR(&P1FLTACON, 0x0087, &PWM1KEY);
    __builtin_write_PWMSFR(&P1FLTBCON, 0x0080, &PWM1KEY);

    //clear the fault interrupt flags
    IFS3bits.FLTA1IF = 0;
    IEC3bits.FLTA1IE = 1;


    //setup comparator on pin2, AN3, C2INB,C1IND,RP1,RB1
    //The comparator monitors the motor current and triggers the PWM fault
    //The comparator pin and FAULT pin are connected externally

    CM1CON = 0x4091;        //setup the comparator module
    CM1CONbits.CPOL = 0;

    //set the comparator reference and trigger point
    CVRCON = 0x0280;        //3.3* (CVR/24)
    CVRCONbits.CVR = 15;    //2.37V -> 0.95A    (Conversion factor is 2.5V/A)


    //map RP7 to comparator C1OUT
    RPOR3bits.RP7R = 1;
    TRISBbits.TRISB7 = 0;

    //enable the comparator
    CM1CONbits.CON =1;

    //enable the MC PWM module
    P1TCONbits.PTEN = 1;
}

/******************************************************************************
* Function:     InitTMR2(void)
*
* Output:       None
*
* Overview:     Initializes the TIMER2 module to operate in free-running
*               up counting mode. The TIMER2 time base is Tcy*64= 4.34 uSec.
*               This timer is used to calculate the motor speed
*
* Note:         None
*******************************************************************************/
void InitTMR2(void)
{                           // Tcy = 67.8 nSec
    TMR2 = 0;               // Resetting TIMER
    PR2 = 0xFFFF;           // Setting TIMER periond to the MAX value
    T2CON = 0x0020;         // internal Tcy*64
}


/******************************************************************************
* Function:     InitTMR1(void)
*
* Output:       None
*
* Overview:     Initializes the TIMER1 module to operate in free-running
*               up counting mode. The TIMER1 time base is Tcy*64 = 4.34 uSec.
*               This timer is used to calculate the commutation delay
*
* Note:         None
*******************************************************************************/
void InitTMR1(void)
{                           // Tcy = 67.8 nSec
    TMR1 = 0;               // Resentting TIMER
    PR1 = 10;               // Intial commutation delay value 43.4 uSeg
    T1CON = 0x0020;         // internal Tcy*64

    IPC0bits.T1IP = 5;      // Set Timer 1 Interrupt Priority Level
    IFS0bits.T1IF = 0;      // Clear Timer 1 Interrupt Flag
    IEC0bits.T1IE = 1;      // Enable Timer1 interrupt
    T1CONbits.TON = 0;      // Debug Enable Timer1

}

