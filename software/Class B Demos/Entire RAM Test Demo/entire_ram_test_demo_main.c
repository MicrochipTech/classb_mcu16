/*************************************************************************
 *  © 2013 Microchip Technology Inc.                                       
 *  
 *  Project Name:    Class B Library Demo
 *  FileName:        Entire RAM Test Demo Main.c
 *  Dependencies:    system.h, classb.h
 *  Processor:       PIC24, dsPIC
 *  Compiler:        XC16
 *  IDE:             MPLAB® X                        
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  Description:     This demo explains how to test RAM memory using class B
 *                   library. For march test a buffer is required to store
 *                   the content of the tested memory. The tested memory
 *                   region must not overlap the buffer area. In this example
 *                   the buffer is located at the beggining of RAM. The 
 *                   entire memory is tested block by block.
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

// 512 bytes buffer at the start of RAM
// (address 0x0850, aligned by 2 bytes).
// RAM from 0x0800 is used by debugger.
// The buffer is used by march tests to store
// the tested memory content.
volatile uint16_t __attribute__((address(RAM_START_ADDRESS), noload)) marchTestBuffer[512/sizeof(uint16_t)]; 

// This is an application task.
void MainApplicationTask();

// This is a RAM test task. The RAM is tested block by block.
CLASSBRESULT RAMTestTask();

int main(void)
{
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

       // Run AM test task periodically.    
       if(RAMTestTask())
       {
           // RAM ERROR IS DETECTED HERE.
           asm volatile ("bra $+0");

       }

       // Run main application.

       MainApplicationTask();
    }
    
    return 0;

} // end of main()    

void MainApplicationTask()
{
    // APPLICATION CODE SHOULD BE HERE.
    SysLedsChaser();
}

CLASSBRESULT RAMTestTask()
{
static uint16_t address = RAM_START_ADDRESS;
uint16_t        length  = sizeof(marchTestBuffer);

    // Length of the last RAM block can be less than the buffer length.
    if(length > (RAM_END_ADDRESS-address))
    {
        length = (RAM_END_ADDRESS-address);
    }

    // March tests buffer is located at the RAM start.
    if((uint16_t*)address == marchTestBuffer)
    {
        // Test buffer for march tests.
        
        // Checker Board test
        if(CLASSB_RAMCheckerboardTest(address,  // start address (must be aligned by word (2 bytes))
                                      length))  // length in bytes (must be aligned by 4 bytes)
        {
            return CLASSB_TEST_FAIL;
        }

        // March C test.
        if(CLASSB_RAMMarchCTest(address, // start address (must be aligned by word (2 bytes))
                                length,  // byte length (must be even number)
                                0,       // buffer is not specified, the memory will be cleared
                                false))  //
        {
            return CLASSB_TEST_FAIL;
        }

        // March C Minus test.
        if(CLASSB_RAMMarchCTest(address, // start address (must be aligned by word (2 bytes))
                                length,  // byte length (must be even number)
                                0,       // buffer is not specified, the memory will be cleared
                                true))   // minus algorithm
        {
            return CLASSB_TEST_FAIL;
        }

        // March B test.
        if(CLASSB_RAMMarchBTest(address, // start address (must be aligned by word (2 bytes))
                                length,  // byte length (must be even number)
                                0))      // buffer is not specified, the memory will be cleared
        {
            return CLASSB_TEST_FAIL;
        }

        // Move to the next memory block.
        address += length;

        // If was last RAM block, move to the RAM start.
        if(address == RAM_END_ADDRESS)
        {
            address = RAM_START_ADDRESS;
        }
     
        return CLASSB_TEST_PASS;
    }

    // Test RAM block saving its content in the buffer.
        
    // Checker Board test
    if(CLASSB_RAMCheckerboardTest(address, // start address (must be aligned by word (2 bytes))
                                  length)) // length in bytes (must be aligned by 4 bytes)
    {
        return CLASSB_TEST_FAIL;
    }

    // March C test.
    if(CLASSB_RAMMarchCTest(address,         // start address (must be aligned by word (2 bytes))
                            length,          // byte length (must be even number)
                            marchTestBuffer, // buffer to save the memory content
                            false))          // 
    {
        return CLASSB_TEST_FAIL;
    }

    // March C Minus test.
    if(CLASSB_RAMMarchCTest(address,         // start address (must be aligned by word (2 bytes))
                            length,          // byte length (must be even number)
                            marchTestBuffer, // buffer to save the memory content
                            true))           // minus algorithm
    {
        return CLASSB_TEST_FAIL;
    }

    // March B test.
    if(CLASSB_RAMMarchBTest(address,          // start address (must be aligned by word (2 bytes))
                            length,           // byte length (must be even number)
                            marchTestBuffer)) // buffer to save the memory content
    {
        return CLASSB_TEST_FAIL;
    }

    // Move to the next memory block.
    address += length;

    // If was last RAM block, move to the RAM start.
    if(address >= RAM_END_ADDRESS)
    {
        address = RAM_START_ADDRESS;
    }
     
    return CLASSB_TEST_PASS;

}
