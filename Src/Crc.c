//==============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/Crc.c $
// $Revision: 9742 $
// $Date: 2011-02-09 10:05:27 +0100 (mer, 09 feb 2011) $
// $Author: lorenzo $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief Crc module.
///
/// \file
/// \par COMMON RESOURCE
/// CRC 8 is used to check both transmission and reception to / from coder B.
/// Therefore different CRC computation functions have been implemented to
/// avoid "common resource" issues. Both functions share the same CRC table.
/// \n\n
/// CRC 16 table is used to check both UID code (read from external eeprom
/// memory) and ROM (Flash) memory content, by means of different functions.
/// Flash memory to be checked goes from address 0x0000 up to address 0xFFFE.
/// At address 0xFFFE the external variable __crc16 is stored which contains
/// the CRC computed by the IAR linker over the entire Flash memory.
///
/// \par MULTIPLE INITIALIZATION
/// Multiple initialization functions are a consequence of the measures to
/// avoid common resource issues, see above
///
/// \par MULTIPLE MANAGEMENT
/// Multiple management functions are a consequence of the measures to
/// avoid common resource issues, see above
///
//  CHANGES doxygen
//
//==============================================================================
#include  <ioavr.h>
#include  <intrinsics.h>
#include "includes.h"
#include "Digital.h"
#include "Crc.h"

/*--------------------------------- Definitions ------------------------------*/

#undef  VAR_STATIC
#define VAR_STATIC static


#define CRC_ERR    ((INT8U) 0xDF) //!< Crc error code



/*---------------------------------- Constants -------------------------------*/

//! CRC8 table for communication coder A - coder B. Polynomial = 0x2F
//! Best choice for 104 data bits length
//! See "http://www.ece.cmu.edu/~koopman/crc/c8_best.txt"
VAR_STATIC INT8U __flash Crc8_Table[256] = {
 0x0,   0x2F,  0x5E,  0x71,  0xBC,  0x93,  0xE2,  0xCD,
 0x57,  0x78,  0x9,   0x26,  0xEB,  0xC4,  0xB5,  0x9A,
 0xAE,  0x81,  0xF0,  0xDF,  0x12,  0x3D,  0x4C,  0x63,
 0xF9,  0xD6,  0xA7,  0x88,  0x45,  0x6A,  0x1B,  0x34,
 0x73,  0x5C,  0x2D,  0x2,   0xCF,  0xE0,  0x91,  0xBE,
 0x24,  0xB,   0x7A,  0x55,  0x98,  0xB7,  0xC6,  0xE9,
 0xDD,  0xF2,  0x83,  0xAC,  0x61,  0x4E,  0x3F,  0x10,
 0x8A,  0xA5,  0xD4,  0xFB,  0x36,  0x19,  0x68,  0x47,
 0xE6,  0xC9,  0xB8,  0x97,  0x5A,  0x75,  0x4,   0x2B,
 0xB1,  0x9E,  0xEF,  0xC0,  0xD,   0x22,  0x53,  0x7C,
 0x48,  0x67,  0x16,  0x39,  0xF4,  0xDB,  0xAA,  0x85,
 0x1F,  0x30,  0x41,  0x6E,  0xA3,  0x8C,  0xFD,  0xD2,
 0x95,  0xBA,  0xCB,  0xE4,  0x29,  0x6,   0x77,  0x58,
 0xC2,  0xED,  0x9C,  0xB3,  0x7E,  0x51,  0x20,  0xF,
 0x3B,  0x14,  0x65,  0x4A,  0x87,  0xA8,  0xD9,  0xF6,
 0x6C,  0x43,  0x32,  0x1D,  0xD0,  0xFF,  0x8E,  0xA1,
 0xE3,  0xCC,  0xBD,  0x92,  0x5F,  0x70,  0x1,   0x2E,
 0xB4,  0x9B,  0xEA,  0xC5,  0x8,   0x27,  0x56,  0x79,
 0x4D,  0x62,  0x13,  0x3C,  0xF1,  0xDE,  0xAF,  0x80,
 0x1A,  0x35,  0x44,  0x6B,  0xA6,  0x89,  0xF8,  0xD7,
 0x90,  0xBF,  0xCE,  0xE1,  0x2C,  0x3,   0x72,  0x5D,
 0xC7,  0xE8,  0x99,  0xB6,  0x7B,  0x54,  0x25,  0xA,
 0x3E,  0x11,  0x60,  0x4F,  0x82,  0xAD,  0xDC,  0xF3,
 0x69,  0x46,  0x37,  0x18,  0xD5,  0xFA,  0x8B,  0xA4,
 0x5,   0x2A,  0x5B,  0x74,  0xB9,  0x96,  0xE7,  0xC8,
 0x52,  0x7D,  0xC,   0x23,  0xEE,  0xC1,  0xB0,  0x9F,
 0xAB,  0x84,  0xF5,  0xDA,  0x17,  0x38,  0x49,  0x66,
 0xFC,  0xD3,  0xA2,  0x8D,  0x40,  0x6F,  0x1E,  0x31,
 0x76,  0x59,  0x28,  0x7,   0xCA,  0xE5,  0x94,  0xBB,
 0x21,  0xE,   0x7F,  0x50,  0x9D,  0xB2,  0xC3,  0xEC,
 0xD8,  0xF7,  0x86,  0xA9,  0x64,  0x4B,  0x3A,  0x15,
 0x8F,  0xA0,  0xD1,  0xFE,  0x33,  0x1C,  0x6D,  0x42,
};

//! CRC8 table for communication coder A - Hic. Polynomial = 0x5B
//! Best choice for 88 data bits length
//! See "http://www.ece.cmu.edu/~koopman/crc/c8_best.txt"
VAR_STATIC INT8U __flash Crc8_Table_Hic[256] = {
 0x0,   0x5B,  0xB6,  0xED,  0x37,  0x6C,  0x81,  0xDA,
 0x6E,  0x35,  0xD8,  0x83,  0x59,  0x2,   0xEF,  0xB4,
 0xDC,  0x87,  0x6A,  0x31,  0xEB,  0xB0,  0x5D,  0x6,
 0xB2,  0xE9,  0x4,   0x5F,  0x85,  0xDE,  0x33,  0x68,
 0xE3,  0xB8,  0x55,  0xE,   0xD4,  0x8F,  0x62,  0x39,
 0x8D,  0xD6,  0x3B,  0x60,  0xBA,  0xE1,  0xC,   0x57,
 0x3F,  0x64,  0x89,  0xD2,  0x8,   0x53,  0xBE,  0xE5,
 0x51,  0xA,   0xE7,  0xBC,  0x66,  0x3D,  0xD0,  0x8B,
 0x9D,  0xC6,  0x2B,  0x70,  0xAA,  0xF1,  0x1C,  0x47,
 0xF3,  0xA8,  0x45,  0x1E,  0xC4,  0x9F,  0x72,  0x29,
 0x41,  0x1A,  0xF7,  0xAC,  0x76,  0x2D,  0xC0,  0x9B,
 0x2F,  0x74,  0x99,  0xC2,  0x18,  0x43,  0xAE,  0xF5,
 0x7E,  0x25,  0xC8,  0x93,  0x49,  0x12,  0xFF,  0xA4,
 0x10,  0x4B,  0xA6,  0xFD,  0x27,  0x7C,  0x91,  0xCA,
 0xA2,  0xF9,  0x14,  0x4F,  0x95,  0xCE,  0x23,  0x78,
 0xCC,  0x97,  0x7A,  0x21,  0xFB,  0xA0,  0x4D,  0x16,
 0x61,  0x3A,  0xD7,  0x8C,  0x56,  0xD,   0xE0,  0xBB,
 0xF,   0x54,  0xB9,  0xE2,  0x38,  0x63,  0x8E,  0xD5,
 0xBD,  0xE6,  0xB,   0x50,  0x8A,  0xD1,  0x3C,  0x67,
 0xD3,  0x88,  0x65,  0x3E,  0xE4,  0xBF,  0x52,  0x9,
 0x82,  0xD9,  0x34,  0x6F,  0xB5,  0xEE,  0x3,   0x58,
 0xEC,  0xB7,  0x5A,  0x1,   0xDB,  0x80,  0x6D,  0x36,
 0x5E,  0x5,   0xE8,  0xB3,  0x69,  0x32,  0xDF,  0x84,
 0x30,  0x6B,  0x86,  0xDD,  0x7,   0x5C,  0xB1,  0xEA,
 0xFC,  0xA7,  0x4A,  0x11,  0xCB,  0x90,  0x7D,  0x26,
 0x92,  0xC9,  0x24,  0x7F,  0xA5,  0xFE,  0x13,  0x48,
 0x20,  0x7B,  0x96,  0xCD,  0x17,  0x4C,  0xA1,  0xFA,
 0x4E,  0x15,  0xF8,  0xA3,  0x79,  0x22,  0xCF,  0x94,
 0x1F,  0x44,  0xA9,  0xF2,  0x28,  0x73,  0x9E,  0xC5,
 0x71,  0x2A,  0xC7,  0x9C,  0x46,  0x1D,  0xF0,  0xAB,
 0xC3,  0x98,  0x75,  0x2E,  0xF4,  0xAF,  0x42,  0x19,
 0xAD,  0xF6,  0x1B,  0x40,  0x9A,  0xC1,  0x2C,  0x77,
};

//! CRC16 table for UID and ROM verification. Polynomial = 0x90D9
//! Good choice for 112 data bits length
//! See "http://www.ece.cmu.edu/~koopman/crc/0xc86c.txt"
VAR_STATIC INT16U __flash Crc16_Table[256] = {
 0x0,    0x90D9, 0xB16B, 0x21B2, 0xF20F, 0x62D6, 0x4364, 0xD3BD,
 0x74C7, 0xE41E, 0xC5AC, 0x5575, 0x86C8, 0x1611, 0x37A3, 0xA77A,
 0xE98E, 0x7957, 0x58E5, 0xC83C, 0x1B81, 0x8B58, 0xAAEA, 0x3A33,
 0x9D49, 0xD90,  0x2C22, 0xBCFB, 0x6F46, 0xFF9F, 0xDE2D, 0x4EF4,
 0x43C5, 0xD31C, 0xF2AE, 0x6277, 0xB1CA, 0x2113, 0xA1,   0x9078,
 0x3702, 0xA7DB, 0x8669, 0x16B0, 0xC50D, 0x55D4, 0x7466, 0xE4BF,
 0xAA4B, 0x3A92, 0x1B20, 0x8BF9, 0x5844, 0xC89D, 0xE92F, 0x79F6,
 0xDE8C, 0x4E55, 0x6FE7, 0xFF3E, 0x2C83, 0xBC5A, 0x9DE8, 0xD31,
 0x878A, 0x1753, 0x36E1, 0xA638, 0x7585, 0xE55C, 0xC4EE, 0x5437,
 0xF34D, 0x6394, 0x4226, 0xD2FF, 0x142,  0x919B, 0xB029, 0x20F0,
 0x6E04, 0xFEDD, 0xDF6F, 0x4FB6, 0x9C0B, 0xCD2,  0x2D60, 0xBDB9,
 0x1AC3, 0x8A1A, 0xABA8, 0x3B71, 0xE8CC, 0x7815, 0x59A7, 0xC97E,
 0xC44F, 0x5496, 0x7524, 0xE5FD, 0x3640, 0xA699, 0x872B, 0x17F2,
 0xB088, 0x2051, 0x1E3,  0x913A, 0x4287, 0xD25E, 0xF3EC, 0x6335,
 0x2DC1, 0xBD18, 0x9CAA, 0xC73,  0xDFCE, 0x4F17, 0x6EA5, 0xFE7C,
 0x5906, 0xC9DF, 0xE86D, 0x78B4, 0xAB09, 0x3BD0, 0x1A62, 0x8ABB,
 0x9FCD, 0xF14,  0x2EA6, 0xBE7F, 0x6DC2, 0xFD1B, 0xDCA9, 0x4C70,
 0xEB0A, 0x7BD3, 0x5A61, 0xCAB8, 0x1905, 0x89DC, 0xA86E, 0x38B7,
 0x7643, 0xE69A, 0xC728, 0x57F1, 0x844C, 0x1495, 0x3527, 0xA5FE,
 0x284,  0x925D, 0xB3EF, 0x2336, 0xF08B, 0x6052, 0x41E0, 0xD139,
 0xDC08, 0x4CD1, 0x6D63, 0xFDBA, 0x2E07, 0xBEDE, 0x9F6C, 0xFB5,
 0xA8CF, 0x3816, 0x19A4, 0x897D, 0x5AC0, 0xCA19, 0xEBAB, 0x7B72,
 0x3586, 0xA55F, 0x84ED, 0x1434, 0xC789, 0x5750, 0x76E2, 0xE63B,
 0x4141, 0xD198, 0xF02A, 0x60F3, 0xB34E, 0x2397, 0x225,  0x92FC,
 0x1847, 0x889E, 0xA92C, 0x39F5, 0xEA48, 0x7A91, 0x5B23, 0xCBFA,
 0x6C80, 0xFC59, 0xDDEB, 0x4D32, 0x9E8F, 0xE56,  0x2FE4, 0xBF3D,
 0xF1C9, 0x6110, 0x40A2, 0xD07B, 0x3C6,  0x931F, 0xB2AD, 0x2274,
 0x850E, 0x15D7, 0x3465, 0xA4BC, 0x7701, 0xE7D8, 0xC66A, 0x56B3,
 0x5B82, 0xCB5B, 0xEAE9, 0x7A30, 0xA98D, 0x3954, 0x18E6, 0x883F,
 0x2F45, 0xBF9C, 0x9E2E, 0xEF7,  0xDD4A, 0x4D93, 0x6C21, 0xFCF8,
 0xB20C, 0x22D5, 0x367,  0x93BE, 0x4003, 0xD0DA, 0xF168, 0x61B1,
 0xC6CB, 0x5612, 0x77A0, 0xE779, 0x34C4, 0xA41D, 0x85AF, 0x1576,
};

/*----------------------------------- Locals --------------------------------*/

VAR_STATIC INT8U Rom_Byte;           ///< Byte read from memory
VAR_STATIC INT8U Reg_Crc8_Rx;        ///< Crc 8 register for data received from B
VAR_STATIC INT8U Reg_Crc8_Tx;        ///< Crc 8 register for data transmitted to B
VAR_STATIC INT8U Reg_Crc8_Rx_Hic;    ///< Crc 8 register for data received from HIC
VAR_STATIC INT8U Reg_Crc8_Tx_Hic;    ///< Crc 8 register for data transmitted to HIC
VAR_STATIC INT16U Reg_Crc16;         ///< Crc 16 register

#pragma diag_suppress=Pm038          // Bypass MISRA pointer use inhibition
///< Pointer to flash memory
__no_init VAR_STATIC INT8U __flash *p     @ "SAFETY_RAM";
__no_init VAR_STATIC INT8U __flash *p_inv @ "SAFETY_RAM_REV";
#pragma diag_error=Pm038

///< ROM CRC register
__no_init VAR_STATIC INT16U Rom_Crc     @ "SAFETY_RAM";
__no_init VAR_STATIC INT16U Rom_Crc_Inv @ "SAFETY_RAM_REV";

/*---------------------------------- Globals ---------------------------------*/
/*---------------------------------- Prototypes -------------------------------*/
static void Save_Error_Code( void );


//------------------------------------------------------------------------------
//
//  DESCRIPTION Initializes CRC of Flash (ROM) memory
/// \remarks    -
///
//------------------------------------------------------------------------------
void Crc16_Flash_Init(void)
{
#pragma diag_suppress=Pm038          // Bypass MISRA pointer use inhibition
    p = (INT8U __flash *) 0;
    p_inv = (INT8U __flash *) 0xFFFF;
#pragma diag_error=Pm038
    Rom_Crc = 0;
    Rom_Crc_Inv = 0xFFFF;
}

//------------------------------------------------------------------------------
//
//  DESCRIPTION Computes and checks CRC of Flash (ROM) memory
/// \remarks    CRC Calculation is performed one byte at time.
///             If CRC result is correct, CRC checking is started over,
///             otherwise an andless loop is entered and execution halted.
///             Flash memory to be checked starts from address 0x0000 and
///             ends where the external variable __crc16 is stored.
///
//------------------------------------------------------------------------------
void Crc16_Flash_Check(void)
{
#pragma diag_suppress=Pm038                     // Bypass MISRA pointer use inhibition
    static INT32U len = (INT32U) & __crc16;     // Flash memory length
#pragma diag_error=Pm038

    Ctrl_Flow_Cnt += CALLED_ROM;

#pragma diag_suppress=Pm038                     // Bypass MISRA pointer use inhibition
    if (len--) {                                // End of memory not reached

        if (((INT16U)p ^ (INT16U)p_inv) == 0xFFFF) {
          Rom_Byte = *p++;                      // Load byte from program memory
          p_inv--;
        } else {
          Shut_Down_TU();                       // called twice to be sure the related
          Shut_Down_TU();                       // hardware flip flop toggles its status
        }

        if ((Rom_Crc ^ Rom_Crc_Inv) == 0xFFFF) {
          asm ("CRC_CALC:");                    // Needed for fault insertion macros
          Rom_Crc = Crc16_Table[(Rom_Crc >> 8) ^ Rom_Byte] ^ (Rom_Crc << 8); // Calculate CRC
          Rom_Crc_Inv = ~Rom_Crc;
        } else {
          Shut_Down_TU();                       // called twice to be sure the related
          Shut_Down_TU();                       // hardware flip flop toggles its status
        }

    } else {                                    // End of memory reached

        if (Rom_Crc != __crc16) {               // Wrong CRC
          asm ("CRC_ERROR:");                   // Needed for fault insertion macros
          Save_Error_Code();
        } else {                                // CRC correct
            if ((Rom_Crc ^ Rom_Crc_Inv) == 0xFFFF) {
                asm ("CRC_OK:");                // Needed for fault insertion macros
                Rom_Crc = 0;                    // ROM OK
                Rom_Crc_Inv = 0xFFFF;
            } else {
              Shut_Down_TU();                   // called twice to be sure the related
              Shut_Down_TU();                   // hardware flip flop toggles its status
            }
            if (((INT16U)p ^ (INT16U)p_inv) == 0xFFFF) {
                p = (INT8U __flash *) 0;        // Restart ROM test
                p_inv = (INT8U __flash *) 0xFFFF;
            } else {
              Shut_Down_TU();                   // called twice to be sure the related
              Shut_Down_TU();                   // hardware flip flop toggles its status
            }
            len = (INT32U) & __crc16;
        }
    }
#pragma diag_error=Pm038

    Ctrl_Flow_Cnt_Inv -= CALLED_ROM;
}

//------------------------------------------------------------------------------
//
//  DESCRIPTION Initializes CRC16 register or get CRC16 register value
/// \param[in]  mode TRUE = initialize, FALSE = get value
/// \return     CRC register value
/// \remarks    -
//
//------------------------------------------------------------------------------
INT16U Crc16_Control(INT8U mode)
{
  if (mode) {
    Reg_Crc16 = 0x0000;         // Initial value for CRC16 calculation is 0
  }
  return (Reg_Crc16);
}

//------------------------------------------------------------------------------
//
//  DESCRIPTION CRC 16 computation/checking
/// \param[in]  data byte to compute CRC upon
/// \return     -
/// \remarks    polynomial = 0x90D9 (x^16 + x^15 + x^12 + x^7 + x^6 + x^4 + x^3 + 1)
//
//------------------------------------------------------------------------------
void Crc16_Compute(INT8U data)
{
  Reg_Crc16 = (Reg_Crc16 << 8) ^ Crc16_Table[(INT8U)(Reg_Crc16 >> 8) ^ data];
}

//------------------------------------------------------------------------------
//
//  DESCRIPTION Initializes / gets CRC8 register value for received data.
/// \param[in]  mode TRUE = initialize, FALSE = get value
/// \return     CRC8 register value
/// \remarks    Separate CRC8 functions and registers are used for transmitted
///             and received data, because they could be calculated at the same
///             time.
///
//------------------------------------------------------------------------------
INT8U Crc8_Control_Rx(INT8U mode)
{
  if (mode) {
    Reg_Crc8_Rx = 0x00;        // Initial value for CRC8 calculation is 0
  }
  return (Reg_Crc8_Rx);
}

//------------------------------------------------------------------------------
//
//  DESCRIPTION CRC 8 computation / checking for received data
/// \param[in]  data byte to compute CRC upon
/// \return     -
/// \remarks    polynomial = 0x2F (x^8 + x^5 + x^3 + x^2 + x + 1)
//
//------------------------------------------------------------------------------
void Crc8_Compute_Rx(INT8U data)
{
  Reg_Crc8_Rx = Crc8_Table[Reg_Crc8_Rx ^ data];
}

//------------------------------------------------------------------------------
//
//  DESCRIPTION Initializes / gets CRC8 register value for transmitted data
/// \param[in]  mode TRUE = initialize, FALSE = get value
/// \return     CRC8 register value
/// \remarks    Separate CRC8 functions and registers are used for transmitted
///             and received data, because they could be calculated at the same
///             time.
///
//------------------------------------------------------------------------------
INT8U Crc8_Control_Tx(INT8U mode)
{
  if (mode) {
    Reg_Crc8_Tx = 0x00;       // Initial value for CRC8 calculation is 0
  }
  return (Reg_Crc8_Tx);
}

//------------------------------------------------------------------------------
//
//  DESCRIPTION    CRC 8 computation / checking for transmitted data
/// \param[in]     data byte to compute CRC upon
/// \return        -
/// \remarks       polynomial = 0x2F (x^8 + x^5 + x^3 + x^2 + x + 1)
//
//------------------------------------------------------------------------------
void Crc8_Compute_Tx(INT8U data)
{
  Reg_Crc8_Tx = Crc8_Table[Reg_Crc8_Tx ^ data];
}

//------------------------------------------------------------------------------
//
//  DESCRIPTION Initializes / gets CRC8 register value for received data from Hic.
/// \param[in]  mode TRUE = initialize, FALSE = get value
/// \return     CRC8 register value
/// \remarks    Separate CRC8 functions and registers are used for transmitted
///             and received data, because they could be calculated at the same
///             time.
///
//------------------------------------------------------------------------------
INT8U Crc8_Control_Rx_Hic(INT8U mode)
{
  if (mode) {
    Reg_Crc8_Rx_Hic = 0x00;        // Initial value for CRC8 calculation is 0
  }
  return (Reg_Crc8_Rx_Hic);
}

//------------------------------------------------------------------------------
//
//  DESCRIPTION    CRC 8 computation / checking for received data from Hic
/// \param[in]     data byte to compute CRC upon
/// \return        -
/// \remarks       polynomial = 0x5B (x^8 + x^6 + x^4 + x^3 + x + 1)
//
//------------------------------------------------------------------------------
void Crc8_Compute_Rx_Hic(INT8U data)
{
  Reg_Crc8_Rx_Hic = Crc8_Table_Hic[Reg_Crc8_Rx_Hic ^ data];
}

//------------------------------------------------------------------------------
//
//  DESCRIPTION Initializes / gets CRC8 register value for transmitted data to HIC
/// \param[in]  mode TRUE = initialize, FALSE = get value
/// \return     CRC8 register value
/// \remarks    Separate CRC8 functions and registers are used for transmitted
///             and received data, because they could be calculated at the same
///             time.
///
//------------------------------------------------------------------------------
INT8U Crc8_Control_Tx_Hic(INT8U mode)
{
  if (mode) {
    Reg_Crc8_Tx_Hic = 0x00;       // Initial value for CRC8 calculation is 0
  }
  return (Reg_Crc8_Tx_Hic);
}

//------------------------------------------------------------------------------
//
//  DESCRIPTION    CRC 8 computation / checking for transmitted data to Hic
/// \param[in]     data byte to compute CRC upon
/// \return        -
/// \remarks       polynomial = 0x5B (x^8 + x^6 + x^4 + x^3 + x + 1)
//
//------------------------------------------------------------------------------
void Crc8_Compute_Tx_Hic(INT8U data)
{
  Reg_Crc8_Tx_Hic = Crc8_Table_Hic[Reg_Crc8_Tx_Hic ^ data];
}


//------------------------------------------------------------------------------
//
//  DESCRIPTION Save crc error code into internal EEprom
/// \return     -
/// \remarks    CRC error code: 0x40
//
//------------------------------------------------------------------------------
static void Save_Error_Code( void )
{
  INT8U failure;
  /************************************************************************/
  /*    the following code is executed only if a critical error happened: */
  /*    no function call has to be executed because stack or registers    */
  /*    could be compromized                                              */
  /************************************************************************/
  __disable_interrupt();                // Disable all interrupts

  #pragma diag_suppress=Pm038           // Bypass MISRA pointer use inhibition

  /**************** read previously stored error code *********************/

  EEAR = (INT16U)&Saved_Failure;        // Set up address register
  EECR |= (1<<EERE);                    // Start eeprom read
  failure = EEDR;                       // Read data register

  /************************************************************************/

  failure &= CRC_ERR;                   // Add new error code

  /**************** write error code into eeprom **************************/

  while ((EECR & (1<<EEWE)) != 0) {     // wait till eeprom is busy
    ;
  }
  EEAR = (INT16U)&Saved_Failure;        // Set up address register
  EEDR = failure;                       // Set up data register
  EECR |= (1<<EEMWE);                   // Enable eeprom write
  EECR |= (1<<EEWE);                    // Start eeprom write

  /************************************************************************/

  #pragma diag_error=Pm038

  DDRD  |= 0x40;                        // Set SHUTDOWN RTX pin as output (PD.6)
  PORTD |= 0x40;                        // Shutdown RTX
  for (; ; ) {
  }
}











