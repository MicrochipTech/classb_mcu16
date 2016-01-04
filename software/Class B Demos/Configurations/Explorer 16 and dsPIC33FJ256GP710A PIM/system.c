/*************************************************************************
 *  © 2013 Microchip Technology Inc.                                       
 *  
 *  Project Name:    Class B Library Demo
 *  FileName:        system.c
 *  Dependencies:    system.h
 *  Processor:       PIC24, dsPIC
 *  Compiler:        XC16
 *  IDE:             MPLAB® X                        
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  Description:     This files contains all hardware depended functions.
 *************************************************************************/
/**************************************************************************
 * MICROCHIP SOFTWARE NOTICE AND DISCLAIMER: You may use this software, and 
 * any derivatives created by any person or entity by or on your behalf, 
 * exclusively with Microchip's products in accordance with applicable
 * software license terms and conditions, a copy of which is provided for
 * your referencein accompanying documentation. Microchip and its licensors 
 * retain all ownership and intellectual property rights in the 
 * accompanying software and in all derivatives hereto. 
 * 
 * This software and any accompanying information is for suggestion only. 
 * It does not modify Microchip's standard warranty for its products. You 
 * agree that you are solely responsible for testing the software and 
 * determining its suitability. Microchip has no obligation to modify, 
 * test, certify, or support the software. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE, ITS INTERACTION WITH 
 * MICROCHIP'S PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY 
 * APPLICATION. 
 * 
 * IN NO EVENT, WILL MICROCHIP BE LIABLE, WHETHER IN CONTRACT, WARRANTY, 
 * TORT (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), STRICT 
 * LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, 
 * SPECIAL, PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, 
 * FOR COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, 
 * HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY 
 * OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWABLE BY LAW, 
 * MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS 
 * SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID 
 * DIRECTLY TO MICROCHIP FOR THIS SOFTWARE. 
 * 
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF 
 * THESE TERMS. 
 *************************************************************************/
#ifndef _SYSTEM_C_
#define _SYSTEM_C_

#include "system.h"

void SysDelayMs(uint16_t delay)
{
    while(delay--)
    {
       SysDelay1Ms();
    }
}

void SysBoardInit(void)
{
   // All inputs are digital.
   AD1PCFGL = 0xffff;
   AD1PCFGH = 0xffff;

   // Run at 32 MHz.
   PLLFBD = 0x1e;

   // Timer 1 will be used for the CPU clock test.
   // The timer must count clock reference pulses.
   // For this demo this is a secondary oscillator running with 32768 Hz watch crystal.   
   T1CON = 0;
   T1CONbits.TCS = 1; // external clock source
   T1CONbits.TON = 1;

   __builtin_write_OSCCONL(OSCCONL|0x0002); // enable secondary oscillator

   RCONbits.SWDTEN = 1; // enable watchdog timer
   ClrWdt();

   SysLedsInit();
   SysLedsChaser();
}

////////////////////////////////////////////////////////////////////////////////
void __attribute__((interrupt, auto_psv)) _DefaultInterrupt ( void )
{
  while (1)
  {
      Nop();
      Nop();
      Nop();
  }
}
////////////////////////////////////////////////////////////////////////////////
void __attribute__((interrupt, auto_psv)) _OscillatorFail ( void )
{
  while (1)
  {
      Nop();
      Nop();
      Nop();
  }
}
////////////////////////////////////////////////////////////////////////////////
void __attribute__((interrupt, auto_psv)) _AddressError ( void )
{
  while (1)
  {
      Nop();
      Nop();
      Nop();
  }
}
////////////////////////////////////////////////////////////////////////////////
void __attribute__((interrupt, auto_psv)) _StackError ( void )
{
  while (1)
  {
      Nop();
      Nop();
      Nop();
  }
}
////////////////////////////////////////////////////////////////////////////////
void __attribute__((interrupt, auto_psv)) _MathError ( void )
{
  while (1)
  {
      Nop();
      Nop();
      Nop();
  }
}

#endif
