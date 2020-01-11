//==============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/Diagnostic.c $
// $Revision: 9742 $
// $Date: 2011-02-09 10:05:27 +0100 (mer, 09 feb 2011) $
// $Author: lorenzo $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief  Diagnostic management module
///
/// \file
///
/// \par GLOBALS
/// Variable Saved_Failure is global because must be accessible by main().
/// Interface functions cannot be used by main() itself becouse it stores failure
/// codes when program flow control failed and thus call stack might be faulty.
///
/// \par COMMON RESOURCE
/// The EEprom can be considered as a shared resource because it can be accessed
/// by: \n
/// - main()
/// - startup code - if self tests fails -
/// - interface routines of this module
///
/// Actually there's no contention because main() and startup code access the
/// eeprom inside a critical section and then enter an endless lopp waiting
/// for system shutdown.
///
//  CHANGES doxygen
//
//==============================================================================

#include <ioavr.h>

#include "includes.h"
#include "Uid.h"
#include "Diagnostic.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_GLOBAL
#   undef VAR_GLOBAL
#endif
#define VAR_GLOBAL

/*----------------------------------- Macros ---------------------------------*/

/*------------------------------------ Enums ---------------------------------*/

/*------------------------------------ Const ---------------------------------*/

/*----------------------------------- Globals --------------------------------*/

//! Error code saved into internal EEPROM
VAR_GLOBAL volatile __eeprom INT8U Saved_Failure;

/*------------------------------------ Locals --------------------------------*/

/*---------------------------------- Prototypes ------------------------------*/

//------------------------------------------------------------------------------
//
//  DESCRIPTION Read failure code from internal E2prom
/// \return     Failure code if eeprom contained a failure code, 0 otherwise
/// \remarks    -
//
//-----------------------------------------------------------------------------
INT8U Read_Failure(void)
{
  INT8U code;

#pragma diag_suppress=Pm038                     // Bypass MISRA pointer use inhibition
  code = ~EEPROM_read((INT16U)&Saved_Failure);  // Read saved failure code, if any
#pragma diag_error=Pm038

  return code;
}

//------------------------------------------------------------------------------
//
//  DESCRIPTION Save failure code into internal eeprom
/// \return     TRUE when failure code saving ends
/// \remarks    -
//
//------------------------------------------------------------------------------
BOOLEAN8 Save_Failure(INT8U code)
{
  BOOLEAN8 saved = TRUE;

  if (EEPROM_busy()) {                                // EEPROM busy
    saved = FALSE;
  } else {
    code = ~code;
#pragma diag_suppress=Pm038                            // Bypass MISRA pointer use inhibition
    EEPROM_write((INT16U)&Saved_Failure, (INT8U)code); // Store error into eeprom
#pragma diag_error=Pm038
  }
  return saved;
}

