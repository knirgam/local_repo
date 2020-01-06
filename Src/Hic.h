//============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/Hic.h $
// $Revision: 9742 $
// $Date: 2011-02-09 10:05:27 +0100 (mer, 09 feb 2011) $
// $Author: lorenzo $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief Hic communication manager
///
/// \file
///
// CHANGES doxygen
//
//============================================================================

/*--------------------------------- Definitions ------------------------------*/

#ifndef HIC_H
#   define HIC_H

/*----------------------------------- Macros ---------------------------------*/

/*------------------------------------ Enums ---------------------------------*/

//! Auxiliary commands, used as indexes for auxiliary commands buffer
enum {
  AUX_CMD_1,
  AUX_CMD_2,
  AUX_CMD_3,
  AUX_CMD_4,
  AUX_CMD_5,
  AUX_CMD_NUM
};

/*------------------------------------ Types ---------------------------------*/

/*----------------------------------- Globals --------------------------------*/

/*---------------------------------- Interface -------------------------------*/

/// @defgroup HIC Hic communication manager
/// @{

void Hic_Init(void);               ///< Hic communication module initialization
BOOLEAN8 Send_Diagnostic(INT8U);   ///< Send diagnostic codes to HIC
void Hic_Manager(void);            ///< Manager of Hic communication module
INT8U Hic_Get_Cmd(INT8U);          ///< Get one byte of auxiliary input commands
INT8U Timp_Read(void);             ///< Get TIMP bit

/// @}

#endif
