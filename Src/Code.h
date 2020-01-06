//============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/Code.h $
// $Revision: 10807 $
// $Date: 2011-11-08 11:29:28 +0100 (mar, 08 nov 2011) $
// $Author: lorenzo $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief Coding module
///
/// \file
///
// CHANGES Modified Check_Digital() and Check_Analog(): 
//         - renamed Digital_Error_A() and Analog_Error_A() respectively
//         - inverted return value (TRUE when there is a mismatch)
//         - removed check of CRC 
//         - removed call to Update_Digital() or Update_Analog()
//         Exported functions Update_Digital() and Update_Analog()
//
//============================================================================

#ifndef CODE_H
#   define CODE_H

/*------------------------------- Definitions --------------------------------*/

/*--------------------------------- Globals ----------------------------------*/

/*---------------------------------- Enums -----------------------------------*/

/*-------------------------------- Interface ---------------------------------*/

/// @defgroup CODE Coding module
/// @{

void Code_Init(void);               ///< Initialize coding module
void Sampling_Init(void);           ///< Initializes data sampling
BOOLEAN8 Exchange_Data(void);       ///< Exchange data between coder A and coder B
BOOLEAN8 Digital_Error_A(void);     ///< Check digital commands
BOOLEAN8 Analog_Error_A(void);      ///< Check analogic commands
void Update_Digital(void);          ///< Update digital commands
void Update_Analog(void);           ///< Update analogic commands
INT8U In_Commands(void);            ///< Get status of safety protected commands
BOOLEAN8 In_Start(void);            ///< Get status of START (D1) command
BOOLEAN8 Receive_BCH(void);         ///< Receive BCH from coder B
BOOLEAN8 Check_BCH(void);           ///< Verify CRC8 of BCH received from coder B
void Send_TLG_Coder_B(void);        ///< Send telegram to coder B
void Send_TLG_RTX(void);            ///< Send telegram + BCH to radio transceiver

/// @}

#endif
