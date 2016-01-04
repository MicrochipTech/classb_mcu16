/*************************************************************************
 *  © 2013 Microchip Technology Inc.                                       
 *  
 *  Project Name:    Class B Library Demo
 *  FileName:        All Tests Demo Main.c
 *  Dependencies:    system.h, classb.h
 *  Processor:       PIC24, dsPIC
 *  Compiler:        XC16
 *  IDE:             MPLAB® X                        
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  Description:     This demo shows an usage example of all class B library
 *                   functions.
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

// 512 bytes buffer at the start of RAM
// (address 0x0850, aligned by 2 bytes).
// RAM from 0x0800 is used by debugger.
// The buffer is used by march tests to store
// the tested memory content.
volatile uint16_t __attribute__((address(0x850), noload)) marchTestBuffer[512/sizeof(uint16_t)];

// Flash CRC Test Data "010203040506"
__prog__ uint16_t __attribute__((space(prog), fillupper(0x01))) FlashCRCData = 0x0203;
__prog__ uint16_t __attribute__((space(prog), fillupper(0x04))) FlashCRCData2 = 0x0506;

// EEPROM CRC Test Data "01020304"
__prog__ uint16_t __attribute__((space(prog))) EERPOMCRCData[2] = {0x0102,0x0304};


volatile uint16_t checkSum;

int main(void)
{
    // Check for startup march tests error.
    // These tests are run before main() function entry if CLASSB_MARCH_C_STARTUP,
    // CLASSB_MARCH_C_MINUS_STARTUP or/and CLASSB_MARCH_B_STARTUP flags are set
    // in classb_config.h file.  
    if(CLASSB_StartupMarchTestGetResult())
    {
       // ERROR IS DETECTED
       asm volatile ("bra $+0");
    }

   // Check for a watchdog timer timeout error.  
   if(RCONbits.WDTO == 1)
   {
       RCONbits.WDTO = 0;
       // ERROR IS DETECTED
       asm volatile ("bra $+0");
   }

   // Initialize board hardware.
   SysBoardInit();

   while(1)
   {   
       // CPU Program Counter test.
       // Should be used with a watchdog timer.

       // Check program counter status everywhere using CLASSB_CPUPCTestGetResult()
       // especially before critical code execution (such as motor on/off, power enable and so on).
       if(CLASSB_CPUPCTestGetResult())
       {
           // ERROR IS DETECTED
           asm volatile ("bra $+0");
       }
       CLASSB_CPUPCTest();
       // Clear watchdog.
       ClrWdt();


       // CPU Registers test.
       if(CLASSB_CPURegistersTest())
       {
           // ERROR IS DETECTED
           asm volatile ("bra $+0");
       }
 
       // RAM tests of the temporary buffer used for march tests.

       // Checker board test
       if(CLASSB_RAMCheckerboardTest(marchTestBuffer,         // start address (must be aligned by word (2 bytes))
                                     sizeof(marchTestBuffer)))// byte length (must aligned by 4 bytes)
       {
           // ERROR IS DETECTED
           asm volatile ("bra $+0");
       }

       // March C destructive test.
       // Interrupts are disabled during this test.
       if(CLASSB_RAMMarchCTest(marchTestBuffer,         // start address (must be aligned by word (2 bytes))
                               sizeof(marchTestBuffer), // byte length (must be even number)
                               0,                       // buffer is not specified, the memory will be cleared
                               false))                  //
       {
           // ERROR IS DETECTED
           asm volatile ("bra $+0");
       }

       // March C Minus destructive test.
       // Interrupts are disabled during this test.
       if(CLASSB_RAMMarchCTest(marchTestBuffer,         // start address (must be aligned by word (2 bytes))
                               sizeof(marchTestBuffer), // byte length (must be even number)
                               0,                       // buffer is not specified, the memory will be cleared
                               true))                   // minus algorithm
       {
           // ERROR IS DETECTED
           asm volatile ("bra $+0");
       }

       // March B  destructive test.
       // Interrupts must are disabled during this test.
       if(CLASSB_RAMMarchBTest(marchTestBuffer,         // start address (must be aligned by word (2 bytes))
                               sizeof(marchTestBuffer), // byte length (must be even number)
                               0))                      // buffer is not specified, the memory will be cleared
       {
           // ERROR IS DETECTED
           asm volatile ("bra $+0");
       }

       // RAM tests memory of the .nbss section       
       // The linker has generated the following map information:
       // Data Memory  [Origin = 0x800, Length = 0x7800]
       //
       // section                    address      alignment gaps    total length  (dec)
       // -------                    -------      --------------    -------------------
       // .nbss                        0x800                   0           0xfa2  (4002)
       //
       //                         Total data memory used (bytes):          0xfa2  (4002) 13%
  
       // Checker board test.
       // Interrupts are disabled during this test.
       if(CLASSB_RAMCheckerboardTest(0x0850+sizeof(marchTestBuffer), // start address (must be aligned by word (2 bytes))
                                     512))   // length in bytes (must be aligned by 4 bytes)
       {
           // ERROR IS DETECTED
           asm volatile ("bra $+0");
       }
 
       // March C non destructive test.
       // !!!The selected memory region must not cross the buffer!!! 
       // Interrupts are disabled during this test.
       if(CLASSB_RAMMarchCTest(0x0850+sizeof(marchTestBuffer), // start address (must be aligned by word (2 bytes))
                               512,             // byte length (must be even number)
                               marchTestBuffer, // buffer to store the tested memory content
                               true))          //
       {
           // ERROR IS DETECTED
           asm volatile ("bra $+0");
       }

       // March C Minus non destructive test.
       // !!!The selected memory region must not cross the buffer!!! 
       // Interrupts are disabled during this test.
       if(CLASSB_RAMMarchCTest(0x0850+sizeof(marchTestBuffer), // start address (must be aligned by word (2 bytes))
                               512,             // byte length (must be even number)
                               marchTestBuffer, // buffer to store the tested memory content
                               true))           // minus algorithm
       {
           // ERROR IS DETECTED
           asm volatile ("bra $+0");
       }

       // March B non destructive test.
       // !!!The selected memory region must not cross the buffer!!! 
       // Interrupts are disabled during this test.
       if(CLASSB_RAMMarchBTest(0x0850+sizeof(marchTestBuffer), // start address (must be aligned by word (2 bytes))
                               512,             // byte length (must be even number)
                               marchTestBuffer))// buffer to store the tested memory content
       {
           // ERROR IS DETECTED
           asm volatile ("bra $+0");
       }


       // Result must be 0xDA6F.
       checkSum = CLASSB_CRCFlashTest(&FlashCRCData,  // data start address
                                       4,            // 4 PC units (addressable bytes) = 6 memory bytes
                                      -1);           // initial seed is 0xffff

       // Result must be 0x9E17.
       checkSum = CLASSB_CRCEEPROMTest(EERPOMCRCData,  // data start address
                                       4,              // 4 PC units (addressable bytes) = 4 memory bytes
                                      -1);             // initial seed is 0xffff


       // Calculate the check sum for interrupt vectors table and .text section.
       // The linker has generated the following map information:
       // Program Memory  [Origin = 0x200, Length = 0x2a9fe]
       //
       // section                    address   length (PC units)   length (bytes) (dec)
       // -------                    -------   -----------------   --------------------
       // .text                        0x200                0xc0           0x120  (288)
       // .text                        0x2c0               0xa80           0xfc0  (4032)
       // .dinit                       0xd40                 0x8             0xc  (12)
       // _03F32480_at_address_0000AAAA    0xaaaa                 0x6             0x9  (9)
       // _03F32400_at_address_00015554   0x15554                 0x6             0x9  (9)
       //
       //                       Total program memory used (bytes):         0x10fe  (4350) 1%

       checkSum = CLASSB_CRCFlashTest(0x000000,  // interrupt vectors table start address
                                      0x000100,  // 256 PC units (addressable bytes)
                                      -1);       // initial seed is 0xffff

       checkSum = CLASSB_CRCFlashTest(0x000200,  // the first part of the .text
                                      0x0000c0,  // length in PC units (addressable bytes)
                                      checkSum); // seed is the previous check sum

       checkSum = CLASSB_CRCFlashTest(0x0002c0,  // the second part of the .text
                                      0x000a80,  // length in PC units (addressable bytes)
                                      checkSum); // seed is the previous check sum

       // Clock test.
       // Interrupts are disabled during this test.
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
