/*************************************************************************
 *  © 2013 Microchip Technology Inc.                                       
 *  
 *  Project Name:    Class B Library Demo
 *  FileName:        Entire Flash Test Demo Main.c
 *  Dependencies:    system.h, classb.h
 *  Processor:       PIC24, dsPIC
 *  Compiler:        XC16
 *  IDE:             MPLAB® X                        
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  Description:     This demo explains how to test flash memory using class B
 *                   library. The 16-bit CRC check sum is calculated for entire
 *                   flash. The CRC procedure is executed block by block.
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

// This is an application task.
void MainApplicationTask();

// This is a flash test task. The RAM is tested block by block.
CLASSBRESULT FlashTestTask();

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
    SysBoardInit();

    // Main application loop. 
    while(1)
    {
       // Clear watchdog.
       ClrWdt();

       // Run Flash test task periodically.      
       if(FlashTestTask())
       {
           // FLASH ERROR IS DETECTED HERE.
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

CLASSBRESULT FlashTestTask()
{
static  uint16_t correctCRC = -1;
static  uint16_t crc     = -1;
static  uint32_t address = FLASH_START_ADDRESS;
// Tested block size in program counter units (addressable bytes).
uint32_t         length  = 512;

    // Length of the last RAM block can be less than the buffer length.
    if(length > (FLASH_END_ADDRESS-address))
    {
        length = (FLASH_END_ADDRESS-address);
    }

    // Calcultate CRC of the flash memory block.
    crc = CLASSB_CRCFlashTest(address, // start address (must be aligned by word (2 bytes))
                              length,  // length in program counter units (addressable bytes, must be even)
                              crc);    // initial/previous CRC to continue calculations  


    // Move to the next memory block.
    address += length;

    // If it was last flash block, move to the flash start.
    // Compare CRC with precalculated value.
    if(address == FLASH_END_ADDRESS)
    {
        if(correctCRC != -1)
        { 
            if(crc != correctCRC)
            {
                return CLASSB_TEST_FAIL;
            }
        }else{
            // Assume that the first cycle CRC is correct.
            correctCRC = crc; 
        }
        address = FLASH_START_ADDRESS; // Move to the flash start.
        crc = -1; // Set initial value for the next cycle.
    }
     
    return CLASSB_TEST_PASS;

}
