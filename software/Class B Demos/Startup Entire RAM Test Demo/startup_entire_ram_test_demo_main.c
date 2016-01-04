/*************************************************************************
 *  © 2013 Microchip Technology Inc.                                       
 *  
 *  Project Name:    Class B Library Demo
 *  FileName:        startup_entire_ram_test_demo_main.c
 *  Dependencies:    system.h, classb.h
 *  Processor:       PIC24, dsPIC
 *  Compiler:        XC16
 *  IDE:             MPLAB® X                        
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  Description:     This demo explains how to test entire RAM memory before 
 *                   main() function entry. These tests are enabled if
 *                   CLASSB_MARCH_C_STARTUP,
 *                   CLASSB_MARCH_C_MINUS_STARTUP or/and
 *                   CLASSB_MARCH_B_STARTUP
 *                   flags are set in classb_config.h file. Entire memory
 *                   march tests can take a while. Watchdog timer should
 *                   provide enough time to comlete the tests. It is not
 *                   possible to debug the startup march tests code portion
 *                   because these tests erase the debugger data in the memory.
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
    // Check for startup march tests error.
    // These tests are run before main() function entry if CLASSB_MARCH_C_STARTUP,
    // CLASSB_MARCH_C_MINUS_STARTUP or/and CLASSB_MARCH_B_STARTUP flags are not zero
    // in classb_config.h file.  
    if(CLASSB_StartupMarchTestGetResult())
    {
       // ERROR IS DETECTED
       asm volatile ("bra $+0");
    }

    // Check for watchdog timer timeout error.  
    if(RCONbits.WDTO == 1)
    {
       RCONbits.WDTO = 0;
       // ERROR IS DETECTED
       asm volatile ("bra $+0");
    }

    // Initialize boards' hardware.
    SysBoardInit();

    // Main application loop. 
    while(1)
    {

       // Clear watchdog.
       ClrWdt();
  
       SysLedsChaser();

    }
    
    return 0;

} // end of main()    
