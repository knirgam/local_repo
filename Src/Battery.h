//============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/Battery.h $
// $Revision: 9742 $
// $Date: 2011-02-09 10:05:27 +0100 (mer, 09 feb 2011) $
// $Author: lorenzo $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief Battery manager
///
/// \file
///
// CHANGES doxygen
//
//============================================================================

#ifndef BATTERY_H
#   define BATTERY_H

/*--------------------------------- Definitions ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*----------------------------------- Globals --------------------------------*/

/*------------------------------------ Enums ---------------------------------*/

/*---------------------------------- Interface -------------------------------*/

/// @defgroup BATTERY Battery manager
/// @{

void Battery_Init(void);           ///< Initialize battery management
void Battery_Manager(void);        ///< Battery manager
INT8U Read_Battery(void);          ///< Read battery voltage

/// @}

#endif
