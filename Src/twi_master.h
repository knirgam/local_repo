//============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/twi_master.h $
// $Revision: 9742 $
// $Date: 2011-02-09 10:05:27 +0100 (mer, 09 feb 2011) $
// $Author: lorenzo $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief TWI driver header file
///
/// \file
///
// CHANGES doxygen
//
//============================================================================

/*--------------------------------- Definitions ------------------------------*/

#ifndef TWI_MASTER_H
#   define TWI_MASTER_H

#   define TWI_ERROR  255

/*------------------------------------ Types ---------------------------------*/

/*----------------------------------- Globals --------------------------------*/

/*---------------------------------- Interface -------------------------------*/

/// @defgroup TWI TWI driver
/// @{

void TWI_Master_Init(void);                           ///< TWI initialization
void TWI_Manager(void);                               ///< TWI manager
INT8U TWI_Read(INT8U, INT16U, INT8U, INT8U*);         ///< Starts a TWI reading operation
INT8U TWI_Write(INT8U, INT16U, INT8U, const INT8U*);  ///< Starts a TWI writing operation

/// @}

#endif
