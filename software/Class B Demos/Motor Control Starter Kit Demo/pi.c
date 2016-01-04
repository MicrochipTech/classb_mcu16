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
//  File:       pi.c
//
//  Includes:
//      void InitPI(tPIParm *pParm,int Kp,int Ki,int Kc,int max,int min,int out)
//
/*****************************************************************************/

#include "pi.h"

void InitPI(tPIParm *pParm,int Kp,int Ki,int Kc,int max,int min,int out)
{
    pParm->qdSum = 0;
    pParm->qKp = Kp;
    pParm->qKi = Ki;
    pParm->qKc = Kc;
    pParm->qOutMax = max;
    pParm->qOutMin = min;
    pParm->qOut = out;

}

void CalcPI( tPIParm *pParm)
{
 /*
;    Error  = Reference - Measurement
;    U  = Sum + Kp * Error
;    if( U > Outmax )
;        Out = Outmax
;    else if( U < OutMin )
;        Out = OutMin
;    else
;        Out = U
;    Exc = U - Out
;    Sum = Sum + Ki * Err - Kc * Exc
*/
    int currentError;
    long U;
    int outTemp;

    //Error  = Reference - Measurement
    currentError = pParm->qInRef - pParm->qInMeas;

    //U  = Sum + Kp * Error
    U = __builtin_mulss(currentError, pParm->qKp);
    U = U + pParm->qdSum;


    //limit the output between the allowed limits
    //pParm->qOut is the PI output
    outTemp = (int)(U>>15);
    if(outTemp >  pParm->qOutMax)
        pParm->qOut=  pParm->qOutMax;
    else if(outTemp < pParm->qOutMin)
        pParm->qOut =  pParm->qOutMin;
    else
        pParm->qOut = outTemp;

    //U = Ki * Err
    U = __builtin_mulss(currentError, pParm->qKi);

    //compute the difference between the limited and not limites output
    //currentError is used as a temporary variable
    currentError = outTemp - pParm->qOut;

    //U = U - Kc * Err = Ki * Err - Kc * Exc
    U -= __builtin_mulss(currentError,  pParm->qKc);

    //Sum = Sum + U = Sum + Ki * Err - Kc * Exc
    pParm->qdSum = pParm->qdSum + U;
}
