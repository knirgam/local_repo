//============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/Uart1.h $
// $Revision: 8862 $
// $Date: 2010-10-29 13:49:03 +0200 (ven, 29 ott 2010) $
// $Author: micheleb $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief Uart 1 driver.
///
/// \file
///
// CHANGES Code inspection: header
//
//============================================================================

#ifndef UART1_H
#   define UART1_H

/*-------------------------------- Definitions -------------------------------*/

/*---------------------------------- Macros ----------------------------------*/

/*------------------------------- Enumerations -------------------------------*/

/*---------------------------------- Types -----------------------------------*/

/*--------------------------------- Globals ----------------------------------*/

/*-------------------------------- Interface ---------------------------------*/

/// @defgroup UART1 Uart 1 driver
/// @{

void Uart1_Init(void);               ///< Uart 1 initialization
BOOLEAN8 Uart1_GetC(INT8U *);        ///< Gets a charater from receive buffer
BOOLEAN8 Uart1_PutC(INT8U);          ///< Puts character into transmission buffer
BOOLEAN8 Uart1_PutS(const INT8U * ); ///< Puts string into transmission buffer
void Uart1_Manager(void);            ///< Uart 1 manager


/// @}

#endif
