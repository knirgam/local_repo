//============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/Digital.h $
// $Revision: 9742 $
// $Date: 2011-02-09 10:05:27 +0100 (mer, 09 feb 2011) $
// $Author: lorenzo $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief Digital commands header module
///
/// \file
///
// CHANGES doxygen
//
//============================================================================

/*--------------------------------- Definitions ------------------------------*/

#ifndef DIGITAL_H
#   define DIGITAL_H

#   undef VAR_GLOBAL
#   define VAR_GLOBAL extern

/*----------------------------------- Macros ---------------------------------*/

/*------------------------------------ Enums ---------------------------------*/

//! Digital commands, used as index for digital commands buffer
enum {
  DIGITAL_SAFETY,
  DIGITAL_DH,
  DIGITAL_DL,
  DIGITAL_D1,
  DIGITAL_D2,
  DIGITAL_D3,
  DIGITAL_CMD_NUM
};

/*------------------------------------ Types ---------------------------------*/

/*----------------------------------- Globals --------------------------------*/

/*---------------------------------- Interface -------------------------------*/

/// @defgroup Digital Digital commands driver
/// @{

void Digital_Init(void);              ///< Initialize digital inputs module
void Digital_Manager(void);           ///< Manages digital input sampling
INT8U Digital_Get_Cmd(INT8U);         ///< Get one byte of digital inputs
BOOLEAN8 Power(void);                 ///< Forces Stateflow to enter ON state
BOOLEAN8 In_Stop(void);               ///< Get status of STOP input
BOOLEAN8 In_CTO(void);                ///< Read Consent To Operate signal from HIC
BOOLEAN8 In_Coder_B_Ready(void);      ///< Read ready signal from coder B
BOOLEAN8 Is_Digital_Fail(void);       ///< Returns TRUE when digital state machine fails
BOOLEAN8 Is_Dig_Reading(void);        ///< Digital state machine is busy sampling inputs
void Coder_A_Ready(BOOLEAN8 mode);    ///< Controls trigger signal from coder A to coder B
void Shut_Down_TU(void);              ///< Shut down power supply
void Shut_Down_RTX(void);             ///< Shut down RTX power only
void Test_OK_On(void);                ///< Activate TEST_OK output
void Led_On(void);                    ///< Turn on external red LED and buzzer
void Led_Off(void);                   ///< Turn off external red LED and buzzer
void Led_Red_On(void);                ///< Turn on internal red LED
void Led_Red_Off(void);               ///< Turn off internal red LED
void Stop_Pulse_On(void);             ///< Enable STOP pulse
void Stop_Pulse_Off(void);            ///< Disable STOP pulse

/// @}

#endif

