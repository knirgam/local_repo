//============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/Uart0.h $
// $Revision: 8860 $
// $Date: 2010-10-29 11:51:43 +0200 (ven, 29 ott 2010) $
// $Author: lorenzo $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief Uart 0 driver.
///
/// \file
///
// CHANGES Code inspection: header
//
//============================================================================

#ifndef UART0_H
#   define UART0_H

/*--------------------------------- Definitions ------------------------------*/

/*----------------------------------- Macros ---------------------------------*/

/*--------------------------------- Enumerations -----------------------------*/

/*------------------------------------ Types ---------------------------------*/

/*----------------------------------- Globals --------------------------------*/

/*---------------------------------- Interface -------------------------------*/

/// @defgroup UART0 Uart 0 driver
/// @{

void Uart0_Init(void);               ///< Initialize UART 0
BOOLEAN8 Uart0_PutC(INT8U);          ///< Writes a character into UART transmission buffer
BOOLEAN8 Uart0_PutS(const INT8U*);   ///< Writes a string into UART transmission buffer
BOOLEAN8 Uart0_GetC(INT8U*);         ///< Gets a single charater from UART reception buffer
BOOLEAN8 Uart0_GetS(INT8U*, INT8U);  ///< Gets a line from UART reception buffer
void Uart0_Start_Tx(void);           ///< Starts UART transmission

/// @}

#endif
