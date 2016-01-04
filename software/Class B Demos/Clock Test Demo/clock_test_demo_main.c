/*************************************************************************
 *  © 2013 Microchip Technology Inc.                                       
 *  
 *  Project Name:    Class B Library Demo
 *  FileName:        Clock Test Demo Main.c
 *  Dependencies:    system.h, classb.h
 *  Processor:       PIC24F, PIC24H, dsPIC
 *  Compiler:        XC16
 *  IDE:             MPLAB® X                        
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  Description:     This demo shows a CPU clock test example. The reference
 *                   clock should be connected to the timer.The timer must be
 *                   initialized by the application code to count the reference
 *                   clock pulses. The period register must be set to the maximum
 *                   value. The address of the timer counter register must
 *                   be specified in classb_config.h using the compile time
 *                   option CLASSB_CLOCK_TEST_TIMER_ADDRESS. There's a second
 *                   compile option  CLASSB_CLOCK_TEST_TIME_MS in the 
 *                   classb_config.h. It defines the test time and can be about
 *                   20 mS if clock is between 1-32MHz and the reference clock
 *                   is 50Hz-33kHz.
 *                   
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

#include "system.h"
#include "classb.h"

int main(void)
{
   // Check for watchdog timer timeout error.  
   if(RCONbits.WDTO == 1)
   {
       RCONbits.WDTO = 0;
       // ERROR IS DETECTED
       asm volatile ("bra $+0");
   }

   // Initialize board hardware.
   // Initialization of the reference clock timer is done HERE. 
   SysBoardInit();

   while(1)
   {    
       // Clear watchdog.
       ClrWdt();

        // Interrupts are disabled during this test.

       // Clock test. The reference clock should be connected to the timer.
       // The timer must be initialized by the application code to count the reference clock
       // pulses. THe address of the timer counter register must be specified 
       // in classb_config.h using compile time option CLASSB_CLOCK_TEST_TIMER_ADDRESS.
       if(CLASSB_ClockTest(32000000, // clock is 32 MHz
                           32768,    // secondary oscillator frequency as reference
                           5))       // 0.5% tolerance limit
       {
           // ERROR IS DETECTED
           asm volatile ("bra $+0");
	   }

       // Display progress.
       SysLedsChaser();

   } // end of while
   
   return 1;
}
