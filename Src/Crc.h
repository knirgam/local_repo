//============================================================================+
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/Crc.h $
// $Revision: 9742 $
// $Date: 2011-02-09 10:05:27 +0100 (mer, 09 feb 2011) $
// $Author: micheleb $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief  Crc calculations module
///
/// \file
///
/// \par GLOBALS
/// Variable __crc16 needs to be declared global because it is defined inside
/// linker options file and computed by IAR compiler.
///
// CHANGES doxygen
//
//============================================================================

/*------------------------------ Definitions ---------------------------------*/

#ifndef CRC_H
#   define CRC_H

#   undef  VAR_GLOBAL
#   define VAR_GLOBAL extern

/*---------------------------- Global variables ------------------------------*/

VAR_GLOBAL INT16U __flash __crc16;  // Value of CRC 16 computed on program memory.
                                    // This variable needs to be global because
                                    // it is computed and allocated by the linker.

/*-------------------------------- Interface ---------------------------------*/

/// @defgroup CRC CRC module
/// @{

void Crc16_Flash_Init(void);       ///< Initializes CRC check of Flash (ROM) memory
void Crc16_Flash_Check(void);      ///< Computes and checks CRC of Flash (ROM) memory
INT16U Crc16_Control(INT8U);       ///< Initializes / returns CRC16 register value
void Crc16_Compute(INT8U);         ///< Computes and checks CRC 16
INT8U Crc8_Control_Rx(INT8U);      ///< Initializes / returns CRC8 register value for received data
void Crc8_Compute_Rx(INT8U);       ///< Computes and checks CRC 8 for received data
INT8U Crc8_Control_Tx(INT8U);      ///< Initializes / returns CRC8 register value for transmitted data
void Crc8_Compute_Tx(INT8U);       ///< Computes and checks CRC 8 for transmitted data
INT8U Crc8_Control_Rx_Hic(INT8U);  ///< Initializes / returns CRC8 register value for received data
void Crc8_Compute_Rx_Hic(INT8U);   ///< Computes and checks CRC 8 for received data
INT8U Crc8_Control_Tx_Hic(INT8U);  ///< Initializes / returns CRC8 register value for transmitted data
void Crc8_Compute_Tx_Hic(INT8U);   ///< Computes and checks CRC 8 for transmitted data

/// @}

#endif
