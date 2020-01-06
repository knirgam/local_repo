//============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/Uid.c $
// $Revision: 9742 $
// $Date: 2011-02-09 10:05:27 +0100 (mer, 09 feb 2011) $
// $Author: lorenzo $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief UID manager
///
/// \file
/// Data organization inside external eeprom memory.
/// \code
/// 16 bytes starting from address 0x0000
///
///  BYTE     0        1        2        3         4
///       +--------+--------+---------+--------+--------+--
///       | <--------UID-------> | <--------UID*------> |  ...
///       +--------+--------+---------+--------+--------+--
///
///  BYTE                   5                         6
///     --+--------------------------------+-------------------+--
///  ...  |TIMP|EM1|EM0|TS4|TS3|TS2|TS1|TS0|MU1|MU0|X|X|X|X|X|X|  ...
///     --+--------------------------------+-------------------+--
///
///  BYTE          7                     14       15
///     --+-----------------+-     -+------------+------------+
///  ...  |X|X|X|X|X|X|X|X|X|  ...  | EE_CRC16_H | EE_CRC16_L |
///     --+-----------------+-     -+------------+------------+
/// \endcode
///
/// - X       : Spare bits
/// - UID     : UID code, 20 bits
/// - UID*    : Inverted and complemented UID code, 20 bits
/// - TIMP    : System type: defines auxiliary commands to be sent
/// - EM1,EM0 : Receiver passive STOP time-out [0.5s, 1.2s, 2s]
/// - TS4-TS0 : Transmitter auto switch off time in minutes
///             (1..31, 0 = Auto switch off disabled)
/// - MU1,MU0 : Multiple unit system configuration
/// - EE_CRC16_H : CRC16, high byte
/// - EE_CRC16_L : CRC16, low byte
///
/// \par COMMON RESOURCES:
/// - ADC driver uses TWI to read battery voltage from external ADC
/// - UID manager uses TWI to read code from external EEPROM
///
/// See TWI manager description for working priciple and contention management.
///
/// \par COMMON RESOURCES: UART 0
/// UART 0 is shared by UID manager and Code module:
/// - UID manager uses UART 0 to send UID information to coder B
/// - Code module uses UART 0 to exchange information with coder B (input
///   samples, telegrams, BCH values)
/// \par
/// Contention is prevented by stateflow: Code functions are called only after
/// UID state machine has completed transmission of uID information to coder B.
///
/// \par COMMON RESOURCES: CRC
/// CRC module is shared by HIC manager, UID manager and Code module.\n
/// See CRC module for explanation of contention management.
///
// CHANGES doxygen
//
//============================================================================

#include  <ioavr.h>

#include "includes.h"
#include "Crc.h"
#include "Uart0.h"
#include "TWI_Master.h"
#include "Digital.h"
#include "Uid.h"

/*--------------------------------- Definitions ------------------------------*/

#undef  VAR_STATIC
#define VAR_STATIC static

#define UID_MASK            (INT8U)0xF0   ///< Mask for last byte of UID
#define TSO_MASK            (INT8U)0x1F   ///< Mask for auto switch-off time
#define MULTI_MASK          (INT8U)0xC0   ///< Mask for Multi-unit bits
#define E2PROM_ADDR         (INT8U)0x50   ///< Slave address of I2C eeprom memory
#define E2PROM_START_ADDR   (INT16U)0     ///< Starting address of I2C eeprom memory
#define E2PROM_RESET_VAL    (INT16U)0xff  ///< EEprom reset value
/*----------------------------------- Macros ----------------------------------*/

/*------------------------------------ Const ----------------------------------*/

VAR_STATIC const INT8U Bit_Mask[8] = {     ///< Bit mask
    0x80,
    0x40,
    0x20,
    0x10,
    0x08,
    0x04,
    0x02,
    0x01
};

/*----------------------------------- Enums ----------------------------------*/

enum {
  EE_UID1 = 0,      ///< UID code [a19,a18,..,a12]
  EE_UID2 = 1,      ///< UID code [a11,a10,.., a4]
  EE_UID3 = 2,      ///< UID code [a3,a2,a1,a0,a0*,a1*,a2*,a3*]
  EE_UID4 = 3,      ///< UID code [ a4*, a5*,..,a11*]
  EE_UID5 = 4,      ///< UID code [a12*,a13*,..,a19*]
  EE_TSO  = 5,      ///< Transmitter auto switch-off time
  EE_MULTI = 6,     ///< Multi-Unit number
  EE_CRC16_H = 14,  ///< CRC 16 high byte
  EE_CRC16_L = 15,  ///< CRC 16 low byte
  EE_DATA_SIZE = 16 ///< Number of bytes read from eeprom
};

/*------------------------------ EEPROM variables ----------------------------*/

//! UID and configuration data stored in internal eeprom
VAR_STATIC __no_init volatile __eeprom INT8U Stored_UID[EE_DATA_SIZE];

/*------------------------------------ Locals --------------------------------*/

VAR_STATIC INT8U EE_Data[EE_DATA_SIZE];         ///< UID from external eeprom
VAR_STATIC INT8U Last_EE_Data[EE_DATA_SIZE];    ///< Last UID from internal eeprom
VAR_STATIC INT8U Auto_Switch_Off;               ///< Auto switch-off time
VAR_STATIC INT8U Multi_Unit;                    ///< Multi-unit bits

//! Default eeprom data: Key code = 0, Toff disabled
VAR_STATIC const INT8U Def_EE_Data[EE_DATA_SIZE] =
{
  0x00,0x00,0x0f,0xff,0xff,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x8d,0x7c
};


///< Buffer for UID code
__no_init VAR_STATIC INT8U Uid_Buffer[UID_SIZE]     @ "SAFETY_RAM";
__no_init VAR_STATIC INT8U Uid_Buffer_Inv[UID_SIZE] @ "SAFETY_RAM_REV";

/*---------------------------------- Prototypes -------------------------------*/

/*-----------------------------------------------------------------------------*/

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Uid module initialization
/// \remarks    -
//
//-----------------------------------------------------------------------------
void Uid_Init(void)
{
  INT8U i;

  for (i = UID1; i < UID_SIZE; i++) {   // Clear UID data buffer
    Uid_Buffer[i] = 0;
    Uid_Buffer_Inv[i] = 0xFF;
  }
  Auto_Switch_Off = 0;                  // Auto switch-off is zero
  Multi_Unit = 0;                       // Multi unit = 0
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Check correctness of UID data
/// \param      p_data pointer to data read from eeprom
/// \return     TRUE if data are correct, FALSE otherwise
/// \remarks    -
//
//-----------------------------------------------------------------------------
BOOLEAN8 Uid_Check_A(const INT8U* p_data)
{
  INT8U i, j;                               // Bit and byte index
  INT8U s_bit, i_bit;                       // Straight and inverted bit of UID
  BOOLEAN8 data_ok = FALSE;                 // Return value

  (void)Crc16_Control(TRUE);                // Initialize CRC16 register
  for (i = 0; i < EE_DATA_SIZE; i++) {      // Calculate CRC16 over eeprom data
    Crc16_Compute(p_data[i]);
  }
  if (Crc16_Control(FALSE) == 0) {          // CRC16 of eeprom data is correct
    data_ok = TRUE;                         // Inform caller
    for (j = EE_UID1; j <= EE_UID3; j++) {  // Inverted and complemented UID == UID*
      for (i = 0; i < 8; i++) {
        s_bit = ((p_data[j] & Bit_Mask[i]) != 0);
        i_bit = (((INT8U)(~p_data[EE_UID5 - j]) & Bit_Mask[7 - i]) != 0);
        if (s_bit != i_bit) {               // Inverted and complemented UID bit != UID* bit
          data_ok = FALSE;                  // Inform caller
        }
      }
    }
  }
  return data_ok;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Check if eeprom is busy
/// \return     TRUE if eeprom is busy, FALSE if ready
/// \remarks    Function must be executed with interrupts disabled and is thus
///             declared "__monitor".
///
///             EEWE bit is cleared by hardware when the write operation has
///             been completed. The user software can poll this bit and wait
///             until it is zero before writing another byte.
///
///             The user should also poll the EEWE bit before starting the read
///             operation. If a write operation is in progress, it is neither
///             possible to read the EEPROM, nor to change the EEAR register.
//
//-----------------------------------------------------------------------------
__monitor BOOLEAN8 EEPROM_busy(void)
{
   BOOLEAN8 busy;

   if ((EECR & (1<<EEWE)) == 0) {    // Check for completion of previous write
     busy = FALSE;                   // Writing ended
   } else {
     busy = TRUE;                    // Writing still in progress
   }
   return busy;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Write a byte into internal eeprom at given address
/// \param      address eeprom address to write to
/// \param      data data to be stored into eeprom
/// \remarks    Function must be executed with interrupts disabled and is thus
///             declared "__monitor".
///
///             Procedure followed for writing to internal EEPROM
///             (the order of steps 3 and 4 is not essential):
///
///             - 1 Wait until EEWE becomes zero (EEPROM_busy() == FALSE)
///             - 2 Wait until SPMEN in SPMCSR becomes zero (not relevant).
///             - 3 Write new EEPROM address to EEAR.
///             - 4 Write new EEPROM data to EEDR.
///             - 5 Write a logical one to the EEMWE bit while writing a logical
///                 zero to EEWE bit in EECR register.
///             - 6 Within four clock cycles after setting EEMWE, write a logical
///                 one to EEWE bit.
///
///             Caution:
///
///             An interrupt between step 5 and step 6 will make the write cycle
///             fail, since the EEPROM Master Write Enable will time-out.
///             It is recommended to have the Global Interrupt Flag cleared
///             during the four last steps to avoid these problems.
//
//-----------------------------------------------------------------------------
__monitor void EEPROM_write(INT16U address, INT8U data)
{
    EEAR = address;                  // Set up address and data registers
    EEDR = data;
    EECR |= (1<<EEMWE);              // Write logical one to EEMWE
    EECR |= (1<<EEWE);               // Start eeprom write by setting EEWE
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Read a byte from internal eeprom
/// \param      address eeprom address to read from
/// \return     byte read from eeprom
/// \remarks    Function must be executed with interrupts disabled and is thus
///             declared "__monitor".
///
///             The EEPROM Read Enable bit EERE is the read strobe signal to
///             the EEPROM. When the correct address is set up in the EEAR
///             register, the EERE bit must be written to a logic one to
///             trigger the EEPROM read.
///
///             The EEPROM read access takes one instruction, and the requested
///             data is available immediately.
///
///             When the EEPROM is read, the CPU is halted for four cycles
///             before the next instruction is executed.
//
//-----------------------------------------------------------------------------
__monitor INT8U EEPROM_read(INT16U address)
{
    EEAR = address;                  // Set up address register
    EECR |= (1<<EERE);               // Start eeprom read by writing EERE
    return EEDR;                     // Return data from data register
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Read UID and other data from internal eeprom
/// \return     TRUE when all data have been read, FALSE if reading is not complete
/// \remarks    For each function call, only one byte is read from eeprom.
//
//-----------------------------------------------------------------------------
BOOLEAN8 Read_Int_UID(void)
{
  static INT8U r_index = 0;
  BOOLEAN8 data_read;
  BOOLEAN8 Uid_Present = FALSE;
  INT8U i = 0;

  data_read = FALSE;
  if (EEPROM_busy() == FALSE) {             // Eeprom ready
    if (r_index < EE_DATA_SIZE) {           // All data not yet read
#pragma diag_suppress=Pm038                 // Bypass MISRA pointer use inhibition
                                            // Read byte from internal eeprom
       Last_EE_Data[r_index] = EEPROM_read((INT16U)(Stored_UID + r_index));
#pragma diag_error=Pm038
       r_index++;                           // Increase byte index
    } else {                                // End of reading
      r_index = 0;                          // Reset index
      data_read = TRUE;                     // Return TRUE
      while( ( i < EE_DATA_SIZE ) && ( Uid_Present == FALSE ) ) {
        if( Last_EE_Data[i] != E2PROM_RESET_VAL ) {
          Uid_Present = TRUE;
        }
        i++;
      }
      if( Uid_Present == FALSE ) {
        for( i=0; i < EE_DATA_SIZE; i++ ) {
          Last_EE_Data[i] = Def_EE_Data[i];
        }
      }
    }
  }
  return data_read;                         //
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Read UID and other data from external eeprom via I2C bus
/// \return     TRUE if all data have been read and are correct, FALSE otherwise
/// \remarks    -
//
//-----------------------------------------------------------------------------
BOOLEAN8 Read_Ext_UID(void)
{
   INT8U i, temp;
   INT8U twi_ans = 0;           // Answer from I2C manager
   BOOLEAN8 data_ok = FALSE;    // All data are read and verified
   //
   // Start reading external eeprom
   //
   twi_ans = TWI_Read(E2PROM_ADDR, E2PROM_START_ADDR, EE_DATA_SIZE, EE_Data);

   if (twi_ans == EE_DATA_SIZE) {               // Reading complete
     if (Uid_Check_A(EE_Data) == TRUE) {        // Verify data
       for (i = UID1; i < UID_SIZE; i++) {      // Data OK
         temp = EE_Data[i];                     // Copy UID into local buffer
         Uid_Buffer[i] = temp;                  //
         Uid_Buffer_Inv[i] = ~temp;             //
       }
       Uid_Buffer[UID3] &= UID_MASK;            // Mask UID*
       Uid_Buffer_Inv[UID3] |= ~UID_MASK;       //
       //
       // Update auto switchoff time
       //
       Auto_Switch_Off = (EE_Data[EE_TSO] & TSO_MASK);
       //
       // Update Multi-Unit bits
       //
       Multi_Unit = ((INT8U)(EE_Data[EE_MULTI] & MULTI_MASK) >> (INT8U)6);
       data_ok = TRUE;
     }
   }
   return data_ok;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Compare data of internal and external eeproms
/// \return     TRUE if data match, FALSE otherwise
/// \remarks    -
//
//-----------------------------------------------------------------------------
BOOLEAN8 Cmp_UID(void)
{
   BOOLEAN8 cmp_ok = TRUE;
   INT8U i;

   for (i = 0; i < EE_DATA_SIZE; i++) {
     if (Last_EE_Data[i] != EE_Data[i]) {  // Data do not match
       cmp_ok = FALSE;                     //
     }
   }
   return cmp_ok;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Store UID into internal eeprom
/// \return     TRUE when writing is complete, FALSE otherwise
/// \remarks    Both Last_EE_Data (which is an array in RAM), and
///             Stored_UID buffer (which is an array in eeprom) are updated.
//
//-----------------------------------------------------------------------------
BOOLEAN8 Store_UID(void)
{
  static INT8U w_index = 0;
  BOOLEAN8 end_store = FALSE;

  if (EEPROM_busy() == FALSE) {     // Internal eeprom ready
    if (w_index < EE_DATA_SIZE) {   // Storing not completed
      //
      // Copy data into internal eeprom buffer
      //
      Last_EE_Data[w_index] = EE_Data[w_index];
      //
      // Copy external eeprom data into internal eeprom
      //
#pragma diag_suppress=Pm038         // Bypass MISRA pointer use inhibition
      EEPROM_write((INT16U)(Stored_UID + w_index), EE_Data[w_index]);
#pragma diag_error=Pm038
      w_index++;                    // Increase write index
      end_store = FALSE;            //
    } else {                        // Storing completed
      w_index = 0;                  // Reset write index
      end_store = TRUE;
    }
  }
  return end_store;                 //
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Send UID to coder B.
/// \remarks    Two signal are used to synchronize sending of UID to coder B.
///
///  Successful transmission of UID to coder B:
/// \code
///                       +------------------+
///                       o--+           +-->|
///  _____________________|  |           |   |____________
///  CODER A TRIGGER      .  |           |
///                       .  |           |
///                       .  |   +----+  |        +-------
///                       .  +-->|    o--+        |
///  _____________________.______|    |___________|
///  CODER B READY        .      .    .           .
///                       .      .    .           .
///  Coder A has read UID..      .    .           .
///                              .    .           .
///  Coder B acknowledges,       .    .           .
///  coder A starts sending UID...    .           .
///                                   .           .
///  Coder B has received UID..........           .
///                                               .
///  UID is correct................................
///
/// \endcode
///  Error during transmission of UID to coder B:
/// \code
///                       +------------------+          +------------------+
///                       o--+           +-->|          o--+           +-->|
///  _____________________|  |           |   |___/ /____|  |           |   |___
///  CODER A TRIGGER      .  |           |              .  |           |
///                       .  |           |              .  |           |
///                       .  |   +----+  |              .  |   +----+  |
///                       .  +-->|    o--+              .  +-->|    o--+
///  _____________________.______|    |__________/ /____.______|    |__________
///  CODER B READY        .      .    .                 .
///                       .      .    .                 .
///  Coder A has read UID..      .    .                 .
///                              .    .                 .
///  Coder B acknowledges,       .    .                 .
///  coder A starts sending UID...    .                 .
///                                   .                 .
///  Coder B has received UID..........                 .
///                                                     .
///  Timeout elapsed,                                   .
///  coder A retries transmission........................
/// \endcode
//
//-----------------------------------------------------------------------------
void Send_UID(void)
{
   INT8U i;

   for (i = EE_UID1; i < EE_DATA_SIZE; i++) {
     (void)Uart0_PutC(EE_Data[i]);  // Send UID to coder B
   }
   Uart0_Start_Tx();                // Start UART transmission
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Gets a byte of UID
/// \param      index UID byte index
/// \return     indexed byte of UID buffer
/// \remarks    If index is out of range, 0 is returned
//
//-----------------------------------------------------------------------------
INT8U Uid_Read(INT8U index)
{
  INT8U uid_byte = 0;

  if (index < UID_SIZE) {
    if ((Uid_Buffer[index] ^ Uid_Buffer_Inv[index]) == 0xFF) {
      uid_byte = Uid_Buffer[index];
    } else {
      Shut_Down_TU();   // called twice to be sure the
      Shut_Down_TU();   // flip flop changes status
    }
  }
  return uid_byte;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Get auto switch-off time
/// \return     0 if auto switch off is disabled, a number in the range 1-31
///             [minutes] otherwise
/// \remarks    -
//
//-----------------------------------------------------------------------------
INT8U Auto_Switchoff_Read(void)
{
   return Auto_Switch_Off;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Get Multi Unit bits
/// \return     0, 1, 2, or 3
/// \remarks    -
//
//-----------------------------------------------------------------------------
INT8U Multi_Unit_Read(void)
{
  return Multi_Unit;
}

