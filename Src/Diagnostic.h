//============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/Diagnostic.h $
// $Revision: 9742 $
// $Date: 2011-02-09 10:05:27 +0100 (mer, 09 feb 2011) $
// $Author: micheleb $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief Diagnostic manager
///
/// \file
///
// CHANGES doxygen
//
//============================================================================

/*--------------------------------- Definitions ------------------------------*/

#ifndef DIAGNOSTIC_H
#   define DIAGNOSTIC_H

/*----------------------------------- Macros ---------------------------------*/

/*------------------------------------ Enums ---------------------------------*/

//! Failure codes
enum {
    NO_FAIL    = 0,  ///< No failures
    DIG_FAIL   = 1,  ///< Digital commands failure
    COMM_FAIL  = 2,  ///< Communication failure
    DATA_FAIL  = 4,  ///< Data mismatch failure
    STOP_FAIL  = 8,  ///< STOP failure
    PROG_FLOW  = 16, ///< Program flow error
    FLASH_ERR  = 32, ///< Flash startup error
    CPU_ERR    = 64, ///< cpu startup error
    RAM_ERR    = 128,///< Ram startup error
};

/*------------------------------------ Types ---------------------------------*/

/*------------------------------- Global variables ---------------------------*/

/*---------------------------------- Interface -------------------------------*/

/// @defgroup DIAGNOSTIC Diagnostic manager
/// @{

void Diagnostic_Init(void);        ///< Diagnostic module initialization
BOOLEAN8 Save_Failure(INT8U);      ///< Save failure code into internal e2prom
INT8U Read_Failure(void);          ///< Read failure code from internal e2prom

/// @}

#endif
