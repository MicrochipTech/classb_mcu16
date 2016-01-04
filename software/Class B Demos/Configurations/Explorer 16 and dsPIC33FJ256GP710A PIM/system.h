/*************************************************************************
 *  © 2013 Microchip Technology Inc.                                       
 *  
 *  Project Name:    Class B Library
 *  FileName:        system.h
 *  Dependencies:    xc.h, stdint.h
 *  Processor:       PIC24, dsPIC
 *  Compiler:        XC16
 *  IDE:             MPLAB® X                        
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  Description:     This header contains hardware depended definitions.
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

#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include <xc.h>
#include <stdint.h>

/****************************************************************************
  System oscillator frequency. 
  ***************************************************************************/
#define CLOCK_FREQUENCY  32000000

#if !defined ( __C30__ )
    #error This system file is for PIC24/dsPIC only.
#endif

#ifdef _SYSTEM_C_
// PRIVATE

// dsPIC33FJ256GP710A CONFIGURATION SETTINGS
_FBS(BWRP_WRPROTECT_OFF & BSS_NO_BOOT_CODE & RBS_NO_BOOT_RAM);
_FSS(SWRP_WRPROTECT_OFF & SSS_NO_SEC_CODE & RSS_NO_SEC_RAM);
_FGS(GWRP_OFF & GCP_OFF);
_FOSCSEL(FNOSC_PRIPLL);
_FOSC(POSCMD_XT & FCKSM_CSECME);
_FWDT(WDTPOST_PS128 & WDTPRE_PR128 & WINDIS_OFF & FWDTEN_OFF);
_FICD(ICS_PGD1 & JTAGEN_OFF);

/****************************************************************************
  Macro:
    SysLedsInit()

  Summary:
    Initializes board's LEDs.

  Description:
    Initializes board's LEDs.
 
  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
  ***************************************************************************/
#define SysLedsInit()  {LATA &= ~0x00ff; TRISA &= ~0x00ff;}

/****************************************************************************
  Macro:
    SysDelay1Ms()

  Summary:
    Delay execution on 1 mS.

  Description:
    Delay execution on 1 mS.
 
  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
  ***************************************************************************/
#define SysDelay1Ms()  {asm volatile("Repeat #16000"); asm volatile("nop");}

#endif

// PUBLIC

// DEVICE RAM
#define RAM_START_ADDRESS   ((uint16_t)0x0850)
#define RAM_END_ADDRESS     ((uint16_t)0x8000)

// DEVICE FLASH
#define FLASH_START_ADDRESS  ((uint32_t)0x000000)
#define FLASH_END_ADDRESS    ((uint32_t)0x02AC00)

// LEDS CONTROL
#define Led0On()     {LATAbits.LATA0 = 1;}
#define Led0Off()    {LATAbits.LATA0 = 0;}

#define Led1On()     {LATAbits.LATA1 = 1;}
#define Led1Off()    {LATAbits.LATA1 = 0;}

#define Led2On()     {LATAbits.LATA2 = 1;}
#define Led2Off()    {LATAbits.LATA2 = 0;}

#define Led3On()     {LATAbits.LATA3 = 1;}
#define Led3Off()    {LATAbits.LATA3 = 0;}

#define Led4On()     {LATAbits.LATA4 = 1;}
#define Led4Off()    {LATAbits.LATA4 = 0;}

#define Led5On()     {LATAbits.LATA5 = 1;}
#define Led5Off()    {LATAbits.LATA5 = 0;}

#define Led6On()     {LATAbits.LATA6 = 1;}
#define Led6Off()    {LATAbits.LATA6 = 0;}

#define Led7On()     {LATAbits.LATA7 = 1;}
#define Led7Off()    {LATAbits.LATA7 = 0;}

#define Led_ALLOn()  {LATA |= 0x00ff;}
#define Led_ALLOff() {LATA &= ~0x00ff;}


/****************************************************************************
  Function:
    void SysDelayMs(uint16_t delay)

  Summary:
    Delay execution on defined time.

  Description:
    Delay execution on defined time.
 
  Precondition:
    None

  Parameters:
    delay  - delay time in mS.

  Returns:
    None.

  Remarks:
    None.
  ***************************************************************************/
void SysDelayMs(uint16_t delay);

/****************************************************************************
  Macro:
    SysLedsChaser()

  Summary:
    Turn on and off all LEDs in sequence.

  Description:
    Turn on and off all LEDs in sequence.
 
  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
  ***************************************************************************/
#define SysLedsChaser() {\
Led0On(); SysDelayMs(20);Led0Off();Led1On(); SysDelayMs(20);Led1Off();Led2On(); SysDelayMs(20);Led2Off();Led3On(); SysDelayMs(20);Led3Off();\
Led4On(); SysDelayMs(20);Led4Off();Led5On(); SysDelayMs(20);Led5Off();Led6On(); SysDelayMs(20);Led6Off();Led7On(); SysDelayMs(20);Led7Off();\
Led7On(); SysDelayMs(20);Led7Off();Led6On(); SysDelayMs(20);Led6Off();Led5On(); SysDelayMs(20);Led5Off();Led4On(); SysDelayMs(20);Led4Off();\
Led3On(); SysDelayMs(20);Led3Off();Led2On(); SysDelayMs(20);Led2Off();Led1On(); SysDelayMs(20);Led1Off();Led0On(); SysDelayMs(20);Led0Off();\
}

/****************************************************************************
  Function:
    void SysBoardInit(void)

  Summary:
    Initializes the board hardware.

  Description:
    Initializes the board hardware.
 
  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
  ***************************************************************************/
void SysBoardInit(void);

#endif
