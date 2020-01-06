//============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/Uid.h $
// $Revision: 9742 $
// $Date: 2011-02-09 10:05:27 +0100 (mer, 09 feb 2011) $
// $Author: micheleb $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief UID manager header file
///
/// \file
///
// CHANGES doxygen
//
//============================================================================

#ifndef UID_H
#   define UID_H

/*--------------------------------- Definitions -----------------------------*/

/*------------------------------------ Types --------------------------------*/

/*----------------------------------- Globals -------------------------------*/

/*------------------------------------ Enums --------------------------------*/

enum {
  UID1 = 0,       ///< Most significant byte of UID
  UID2 = 1,
  UID3 = 2,       ///< Least significant bits of UID
  UID_SIZE = 3     ///< Number of bytes for UID
};

/*---------------------------------- Interface ------------------------------*/

/// @defgroup UID UID manager
/// @{

void Uid_Init(void);               ///< UID module initialization
BOOLEAN8 Read_Int_UID(void);       ///< Read UID from internal eeprom
BOOLEAN8 Read_Ext_UID(void);       ///< Read UID from external eeprom via I2C bus
BOOLEAN8 Cmp_UID(void);            ///< Compare UID of internal and external eeproms
BOOLEAN8 Store_UID(void);          ///< Store UID into internal eeprom
void Send_UID(void);               ///< Send UID to coder B
BOOLEAN8 Uid_Check_A(const INT8U* p_data); ///< Check correctness of UID data
INT8U Uid_Read(INT8U index);       ///< Get bytes of UID code
INT8U Auto_Switchoff_Read(void);   ///< Get auto switch-off delay
INT8U Multi_Unit_Read(void);       ///< Get Multi Unit bits
BOOLEAN8 EEPROM_busy(void);        ///< Check if internal eeprom is busy
void EEPROM_write(INT16U, INT8U);  ///< Write a byte into internal eeprom
INT8U EEPROM_read(INT16U);         ///< Read byte from internal eeprom


/// @}

#endif
