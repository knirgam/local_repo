//============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/Stop_Monitor.h $
// $Revision: 9742 $
// $Date: 2011-02-09 10:05:27 +0100 (mer, 09 feb 2011) $
// $Author: lorenzo $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief Stop Monitor module
///
/// \file
///
// CHANGES doxygen
//
//============================================================================

#ifndef STOP_MONITOR_H
#   define STOP_MONITOR_H

/*--------------------------------- Definitions ------------------------------*/

/*------------------------------------ Enum ----------------------------------*/

//! Stop monitor error codes
enum {
    STOP_NO_FAIL,
    STOP_FAIL_PULSE_MISSING,
    STOP_FAIL_PULSE_WRONG
};

/*---------------------------------- Constants -------------------------------*/

/*------------------------------- Local variables ----------------------------*/

/*------------------------------ Global variables ----------------------------*/

/*---------------------------------- Interface -------------------------------*/

/// @defgroup STOP_MONITOR STOP monitor module
/// @{

void Stop_Monitor_Init(void);        ///< STOP monitor initialization
void Stop_Monitor_Manager(void);     ///< Manages the B_STOP pulse monitoring
INT8U Is_Stop_Mon_Fail(void);        ///< Stop monitor is in error status

/// @}

/*----------------------------------------------------------------------------*/

#endif
