//============================================================================+
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/Code.c $
// $Revision: 10807 $
// $Date: 2011-11-08 11:29:28 +0100 (mar, 08 nov 2011) $
// $Author: micheleb $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief Module for data/commands management and telegram coding.
///
/// \file
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
/// \par MAGIC NUMBERS
/// Explicit numbers are used instead of #define'd macros throughout the code
/// wherever it's been considered easier to understand.
///
/// \par NESTED IF
/// Function In_Commands() uses nested if instead of switch statement in order
/// to explicitate the priorities between conditions.
///
// CHANGES Modified Check_Digital() and Check_Analog(): 
//         - renamed Digital_Error_A() and Analog_Error_A() respectively
//         - inverted return value (TRUE when there is a mismatch)
//         - removed check of CRC 
//         - removed call to Update_Digital() or Update_Analog()
//         Exported functions Update_Digital() and Update_Analog()
//
//============================================================================*/

#include  <ioavr.h>

#include "includes.h"
#include "tmwtypes.h"
#include "Uart0.h"
#include "Spi.h"
#include "Digital.h"
#include "Adc.h"
#include "Crc.h"
#include "Uid.h"
#include "Hic.h"
#include "Logic.h"
#include "Code.h"

/*--------------------------------- Definitions ------------------------------*/

#undef  VAR_STATIC
#define VAR_STATIC static

#define VTAP_MIN        (INT8U)0x7A ///< Minimum center tap voltage
#define VTAP_MAX        (INT8U)0x84 ///< Maximum center tap voltage
#define VTAP_NOM        (INT8U)0x7F ///< Nominal center tap voltage
#define V_MIN           (INT8U)0x16 ///< Voltage threshold for joystick connected
#define UPPER_ANALOG    (INT16)20   ///< Upper limit of analog values
#define LOWER_ANALOG    (INT16)20   ///< Lower limiy of analog values
#define MASK_D2_D8      (INT8U)0xFE ///< Mask for D2-D8 digital input
#define MASK_D9_D10     (INT8U)0x03 ///< Mask for D9-D10 digital input
#define MASK_D11_D16    (INT8U)0xFC ///< Mask for D11-D16 digital input
#define MASK_D17_D20    (INT8U)0x0F ///< Mask for D17-D20 digital input
#define MASK_D1         (INT8U)0x01 ///< Mask for D1 digital input
#define MASK_SAF        (INT8U)0x01 ///< Mask for SAFETY input
#define MASK_STOP       (INT8U)0x02 ///< Mask for STOP input
#define MASK_TLG_SAF    (INT8U)0x30 ///< Mask for SAFETY bit into telegram
#define BAT_LOW_SHIFT   (INT8U)3    ///< Position of battery status bit, inside CNTRL_1 byte of telegram
#define TIMP_SHIFT      (INT8U)2    ///< Position of TIMP bit, inside CNTRL_1 byte of telegram
#define MUX_SHIFT       (INT8U)6    ///< Position of MUX bit, inside CNTRL_2 byte of telegram

/*------------------------------------ Enums ---------------------------------*/

//! States of coder state machine
enum {
  ANALOG_EXCHANGE = 0,
  DIGITAL_EXCHANGE = 1,
  WAIT_END_EXCHANGE = 2
};

//! Results of initial check for active commands
enum {
  CMD_OFF = 0,      ///< No command active
  CMD_SAFETY = 1,   ///< SAFETY command active
  CMD_D2D10 = 2,    ///< D2 - D10 commands active
  CMD_DIR = 3,      ///< DIRECTION commands active
  CMD_D11D20 = 4,   ///< D11 - D20 commands active
  CMD_ANALOG = 5    ///< Joysticks active
};

//! Telegram bytes indexes
enum {
  UID_1 = 0,
  UID_2 = 1,
  UID_3 = 2,
  CNTRL_1 = UID_3,
  CNTRL_2 = 3,
  SAFETY = 4,
  DIRECTION_H = 5,
  DIRECTION_L = 6,
  DIGITAL_1 = 7,
  DIGITAL_2 = 8,
  DIGITAL_3 = 9,
  DIGITAL_4 = 10,
  DIGITAL_5 = 11,
  ANALOG_8 = 12,
  ANALOG_7 = 13,
  ANALOG_6 = 14,
  ANALOG_5 = 15,
  ANALOG_4 = 16,
  ANALOG_3 = 17,
  ANALOG_2 = 18,
  ANALOG_1 = 19,
  DATA_FIELD_LENGTH = 20,
  TLG_CRC8 = DATA_FIELD_LENGTH,
  TLG_SIZE = 21
};

//! BCH bytes indexes
enum {
  BCH_1 = 0,
  BCH_2 = 1,
  BCH_3 = 2,
  BCH_4 = 3,
  BCH_5 = 4,
  BCH_6 = 5,
  BCH_7 = 6,
  BCH_FIELD_LENGTH = 7,
  BCH_CRC8 = BCH_FIELD_LENGTH,
  BCH_SIZE = 8
};

//! Command bytes indexes
enum {
  CODED_ANALOG_1 = 0,
  CODED_ANALOG_2 = 1,
  CODED_ANALOG_3 = 2,
  CODED_ANALOG_4 = 3,
  CODED_ANALOG_5 = 4,
  CODED_ANALOG_6 = 5,
  CODED_ANALOG_7 = 6,
  CODED_ANALOG_8 = 7,
  CODED_SAFETY = 8,
  CODED_DIRECTION_H = 9,
  CODED_DIRECTION_L = 10,
  CODED_DIGITAL_1 = 11,
  CODED_DIGITAL_2 = 12,
  CODED_DATA_LEN = 13,
  CODED_CRC8 = CODED_DATA_LEN,
  CODED_SIZE = 14
};

/*----------------------------------- Macros ---------------------------------*/

/*---------------------------------- Constants -------------------------------*/

VAR_STATIC const INT8U Bit_Mask[8] = {    ///< Bit mask
    0x80,
    0x40,
    0x20,
    0x10,
    0x08,
    0x04,
    0x02,
    0x01
};

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC INT8U Exchange_State;            ///< Coder state machine status variable
VAR_STATIC INT8U Coder_A_Data[CODED_SIZE];  ///< Buffer for data to be sent to coder B
VAR_STATIC INT8U Coder_B_Data[CODED_SIZE];  ///< Buffer for data received from Coder B
VAR_STATIC INT8U Coder_A_Data_Wi;           ///< Write index for coder A data buffer
VAR_STATIC INT8U Coder_B_Data_Wi;           ///< Write index for coder B data buffer
VAR_STATIC INT8U TLG_Buffer[TLG_SIZE];      ///< Buffer for telegram to be sent to RTX
VAR_STATIC INT8U BCH_Buffer[BCH_SIZE];      ///< Buffer for BCH code received from coder B
VAR_STATIC INT8U BCH_Write_Index;           ///< Write index for BCH buffer
VAR_STATIC INT8U Analog_Status;             ///< Status of proportional commands
VAR_STATIC BOOLEAN8 Flg_First_Sample;       ///< First sampling of analog inputs
VAR_STATIC BOOLEAN8 Flg_End_Rx;             ///< End of reception of coder B data


/*--------------------------------- Prototypes -------------------------------*/

static void Send_Coder_A_Data(INT8U ch);
static BOOLEAN8 Receive_Coder_B_Data(void);

/*----------------------------------------------------------------------------*/

//----------------------------------------------------------------------------
//
//  DESCRIPTION Initialize coding module
/// \return     -
/// \remarks    -
//
//----------------------------------------------------------------------------
void Code_Init(void)
{
  Analog_Status = 0xFF;         // Joystick connected
  Flg_First_Sample = TRUE;      // First analogic input sample
}


//----------------------------------------------------------------------------
//
//  DESCRIPTION Initializes data sampling
/// \return     -
/// \remarks    -
//
//----------------------------------------------------------------------------
void Sampling_Init(void)
{
   Coder_A_Data_Wi = 0;                 // Clear write index of coder A data
   Coder_B_Data_Wi = 0;                 // Clear write index of coder B data
   (void)Crc8_Control_Tx(TRUE);         // Initialize CRC for data to coder B
   Start_Sampling();                    // Start ADC sampling
   Exchange_State = ANALOG_EXCHANGE;    // Initialize state machine
   Flg_End_Rx = FALSE;
}


//----------------------------------------------------------------------------
//
//  DESCRIPTION  Exchange data between coder A and coder B
/// \return      TRUE when all data have been exchanged, FALSE otherwise
/// \remarks     -
//
//----------------------------------------------------------------------------
BOOLEAN8 Exchange_Data(void)
{
  BOOLEAN8 end_tx = FALSE;
  INT8U tmp_data, i;

  switch (Exchange_State) {
    case ANALOG_EXCHANGE:                           // Exchange analog commands with coder B
      while (Adc_Get_Cmd(&tmp_data)) {              // ADC sample is ready
        Send_Coder_A_Data(tmp_data);                // Send it to coder B
        Crc8_Compute_Tx(tmp_data);                  // Calculate CRC on transmitted byte
      }
      if ((Is_Sampling() == FALSE) &&               // End of sampling
          (Coder_A_Data_Wi > CODED_ANALOG_8)) {     // All samples transmitted to coder B
        Exchange_State = DIGITAL_EXCHANGE;          // Next state
      }
      break;

    case DIGITAL_EXCHANGE:                          // Exchange safe commands with coder B
      if (Is_Dig_Reading() == FALSE) {              // End of digital input sampling
        for (i = DIGITAL_SAFETY; i <= DIGITAL_D2; i++) {
          tmp_data = Digital_Get_Cmd(i);            // Get a byte of digital commands
          Send_Coder_A_Data(tmp_data);              // Send byte to coder B
          Crc8_Compute_Tx(tmp_data);                // Calculate CRC on transmitted byte
        }
        tmp_data = Crc8_Control_Tx(FALSE);          // Get CRC of transmitted bytes
        Send_Coder_A_Data(tmp_data);                // Send CRC to coder B
        Exchange_State = WAIT_END_EXCHANGE;         // Next state
      }
      break;

    case WAIT_END_EXCHANGE:
      end_tx = TRUE;                                // All data transmitted
      break;

    default:
      break;
  }
  Flg_End_Rx |= Receive_Coder_B_Data();             // Keep receiving data from coder B
  return (BOOLEAN8)((end_tx == TRUE) &&             // End of transmitting and receiving
                    (Flg_End_Rx == TRUE));
}


//----------------------------------------------------------------------------
//
//  DESCRIPTION Send a byte to coder B
/// \param      ch byte to be sent
/// \remarks    -
//
//----------------------------------------------------------------------------
static void Send_Coder_A_Data(INT8U ch)
{
   (void)Uart0_PutC(ch);                  // Send byte to coder B via UART 0
   Coder_A_Data[Coder_A_Data_Wi++] = ch;  // Store byte into data buffer
   if (Coder_A_Data_Wi == CODED_SIZE) {   // Limit array index
     Coder_A_Data_Wi = CODED_SIZE - 1;
   }
   Uart0_Start_Tx();
}

//----------------------------------------------------------------------------
//
//  DESCRIPTION Receives a byte from coder B
/// \return     TRUE when all expected bytes have been read, FALSE otherwise.
/// \remarks    CRC8 is computed on each received byte.
///             Function returns either because all expected bytes have been
///             received or because UART buffer is empty. This prevents
///             caller functions to be blocked for too long.
//
//----------------------------------------------------------------------------
static BOOLEAN8 Receive_Coder_B_Data(void)
{
  BOOLEAN8 data_received = FALSE;               // Not all bytes received yet
  BOOLEAN8 uart_empty = FALSE;                  // UART buffer is not yet empty
  INT8U ch;

  while ((!uart_empty) && (!data_received)) {   // UART got data and not all bytes received
    if (Coder_B_Data_Wi == 0) {                 // Received first byte
      (void)Crc8_Control_Rx(TRUE);              // Initialize CRC register
    }
    if (Uart0_GetC(&ch) == TRUE) {              // Get new byte from UART buffer
      Coder_B_Data[Coder_B_Data_Wi++] = ch;     // Copy byte in the local buffer
      Crc8_Compute_Rx(ch);                      // Compute CRC on received byte
      if (Coder_B_Data_Wi == CODED_SIZE) {      // Received all expected bytes
        Coder_B_Data_Wi = 0;                    // Clear write index
        data_received = TRUE;                   // Inform caller
      }
    } else {                                    // No more bytes in the UART
      uart_empty = TRUE;                        // Force function exit
    }
  }
  return data_received;                         // Inform caller
}


//----------------------------------------------------------------------------
//
//  DESCRIPTION Compares digital commands read by coder A and coder B
/// \return     TRUE if data of coder A and B are different, FALSE otherwise.
/// \remarks    -
//
//----------------------------------------------------------------------------
BOOLEAN8 Digital_Error_A(void)
{
  BOOLEAN8 differ = FALSE;

  if ((Coder_A_Data[CODED_DIRECTION_H] != Coder_B_Data[CODED_DIRECTION_H]) ||
      (Coder_A_Data[CODED_DIRECTION_L] != Coder_B_Data[CODED_DIRECTION_L]) ||
      (Coder_A_Data[CODED_DIGITAL_1] != Coder_B_Data[CODED_DIGITAL_1]) ||
      (Coder_A_Data[CODED_DIGITAL_2] != Coder_B_Data[CODED_DIGITAL_2])) {
      differ = TRUE;
  }

  return differ;
}


//----------------------------------------------------------------------------
//
//  DESCRIPTION Compares analogic commands read by coder A and coder B
/// \return     TRUE if data of coder A and B are different, FALSE otherwise.
/// \remarks    -
//
//----------------------------------------------------------------------------
BOOLEAN8 Analog_Error_A(void)
{
  BOOLEAN8 differ = FALSE;
  INT8U i;

#ifndef DEBUG_JTAG                      // Normal mode: check all A1-A8 commands
   for (i = CODED_ANALOG_1; i <= CODED_ANALOG_8; i++) {
      if (((INT16)Coder_B_Data[i] < ((INT16)Coder_A_Data[i] - LOWER_ANALOG)) ||
          ((INT16)Coder_B_Data[i] > ((INT16)Coder_A_Data[i] + UPPER_ANALOG))) {
        differ = TRUE;                // Analog data mismatch
      }
    }
#else                                   // JTAG mode: check only A1-A4 commands
    for (i = CODED_ANALOG_1; i < CODED_ANALOG_5; i++) {
      if (((INT16) Coder_B_Data[i] < ((INT16) Coder_A_Data[i] - LOWER_ANALOG)) ||
         ((INT16) Coder_B_Data[i] > ((INT16) Coder_A_Data[i] + UPPER_ANALOG))) {
        differ = TRUE;                // Analog data mismatch
      }
    }
#endif

  return differ;
}


//----------------------------------------------------------------------------
//
//  DESCRIPTION Update digital data field of telegram
/// \remarks    SAFETY command is active if SAFETY input is active or if at
///             least one direction command is active.
///             To be activated, SAFETY command must be agreed upon by coder A
///             and B.
///             STOP command is active if STOP button is pressed i.e. if STOP
///             input is at logic level 0. To be activated, STOP command need
///             not to be agreed by coder A and B, it is sufficient that A or
///             B detects that STOP input is at logic level 0.
//
//----------------------------------------------------------------------------
void Update_Digital(void)
{
  INT8U safety_A, safety_B, stop_A, stop_B;

  //
  // Insert UID code in the telegram
  //
  TLG_Buffer[UID_1] = Uid_Read(UID1);
  TLG_Buffer[UID_2] = Uid_Read(UID2);
  TLG_Buffer[UID_3] = Uid_Read(UID3);
  //
  // Insert CONTROL bits in the telegram
  //
  TLG_Buffer[CNTRL_1] |= (INT8U)((INT8U)Is_Battery_Low() << BAT_LOW_SHIFT) +
                                 (INT8U)(Timp_Read() << TIMP_SHIFT) +
                                        Multi_Unit_Read();
  TLG_Buffer[CNTRL_2] = (INT8U)(Get_TLG_Mux() << MUX_SHIFT) +
                                Get_Timestamp();
  //
  // Update status of SAFETY command
  //
  safety_A = (((Coder_A_Data[CODED_SAFETY] & MASK_SAF) != 0) ||
               (Coder_A_Data[CODED_DIRECTION_H]        != 0) ||
               (Coder_A_Data[CODED_DIRECTION_L]        != 0)) ? 1 : 0;
  safety_B = (((Coder_B_Data[CODED_SAFETY] & MASK_SAF) != 0) ||
               (Coder_B_Data[CODED_DIRECTION_H]        != 0) ||
               (Coder_B_Data[CODED_DIRECTION_L]        != 0)) ? 1 : 0;
  //
  // Update status of STOP command
  //
  stop_A = (INT8U)(Coder_A_Data[CODED_SAFETY] & MASK_STOP);
  stop_B = (INT8U)(Coder_B_Data[CODED_SAFETY] & MASK_STOP);
  //
  // Insert STOP and SAFETY commands in the telegram, as follows:
  // |STOPA|STOPB|SAFA|SAFB|x|x|x|x|
  //
  TLG_Buffer[SAFETY] = (INT8U)(stop_A << 6) + (INT8U)(stop_B << 5) +
                       (INT8U)(safety_A << 5) + (INT8U)(safety_B << 4);
  //
  // Insert DIRECTION commands in the telegram
  //
  TLG_Buffer[DIRECTION_H] = (Coder_A_Data[CODED_DIRECTION_H] & Coder_B_Data[CODED_DIRECTION_H]);
  TLG_Buffer[DIRECTION_L] = (Coder_A_Data[CODED_DIRECTION_L] & Coder_B_Data[CODED_DIRECTION_L]);
  //
  // Insert DIGITAL commands in the telegram
  //
  TLG_Buffer[DIGITAL_1]  = (Coder_A_Data[CODED_DIGITAL_1]  & Coder_B_Data[CODED_DIGITAL_1]);
  TLG_Buffer[DIGITAL_2]  = (Coder_A_Data[CODED_DIGITAL_2]  & Coder_B_Data[CODED_DIGITAL_2]);
  //
  // Insert AUXILIARY commands in the telegram
  //
  if (Get_TLG_Mux() == 0) {                           // MUX bit == 0
    TLG_Buffer[DIGITAL_3] = Digital_Get_Cmd(DIGITAL_D3) + Hic_Get_Cmd(AUX_CMD_1);
    TLG_Buffer[DIGITAL_4] = Hic_Get_Cmd(AUX_CMD_2);
    TLG_Buffer[DIGITAL_5] = Hic_Get_Cmd(AUX_CMD_3);
  } else {                                            // MUX bit == 1
    TLG_Buffer[DIGITAL_3] = Hic_Get_Cmd(AUX_CMD_4);
    TLG_Buffer[DIGITAL_4] = Hic_Get_Cmd(AUX_CMD_5);
    TLG_Buffer[DIGITAL_5] = Adc_Get_Aux9();
  }
}


//----------------------------------------------------------------------------
//
//  DESCRIPTION  Update analog data field of telegram
/// \remarks     -
//
//----------------------------------------------------------------------------
void Update_Analog(void)
{
  INT8U i;

  for (i = 0; i <= CODED_ANALOG_8; i++) {             // Update analogic
    //
    // Determine status of joysticks (missing / connected).
    // This is performed only once at first sampling.
    //
    if (Flg_First_Sample == TRUE) {                   // Update only at first sampling
      if (Coder_B_Data[CODED_ANALOG_8 - i] < V_MIN) { // Voltage below threshold
        Analog_Status &= ~Bit_Mask[i];                // Joystick is missing
      } else {                                        // Voltage above threshold
        Analog_Status |= Bit_Mask[i];                 // Joystick is connected
      }
    }
    //
    // Update analogic commands.
    //
    if ((Analog_Status & Bit_Mask[i]) != 0) {         // Joystick is connected
                                                      // Use data from code B
      TLG_Buffer[ANALOG_8 + i] = Coder_B_Data[CODED_ANALOG_8 - i];
    } else {                                          // Joystick is missing
      TLG_Buffer[ANALOG_8 + i] = VTAP_NOM;            // Use standstill value
    }
  }
  Flg_First_Sample = FALSE;
}


//----------------------------------------------------------------------------
//
//  DESCRIPTION Receives BCH from coder B
/// \return     TRUE when all expected bytes have been read, FALSE otherwise.
/// \remarks    CRC8 is computed on each received byte.
///             Function exits either because UART buffer is empty or because
///             BCH_SIZE bytes have been read from UART buffer. This prevents
///             caller functions from being blocked for too long.
//
//----------------------------------------------------------------------------
BOOLEAN8 Receive_BCH(void)
{
  INT8U ch;
  BOOLEAN8 uart_empty;
  BOOLEAN8 data_received;

  uart_empty = FALSE;                           // UART buffer is not yet empty
  data_received = FALSE;                        // Not all bytes received yet

  while ((!uart_empty) && (!data_received)) {   // UART got data and not all bytes received
    if (BCH_Write_Index == 0) {                 // Received first byte
      (void)Crc8_Control_Rx(TRUE);              // Initialize CRC register
    }
    if (Uart0_GetC(&ch) == TRUE) {              // Get new byte from UART buffer
      BCH_Buffer[BCH_Write_Index++] = ch;       // Copy byte in the local buffer
      Crc8_Compute_Rx(ch);                      // Compute CRC8 on new byte
      if (BCH_Write_Index == BCH_SIZE) {        // Received all expected bytes
        BCH_Write_Index = 0;                    // Clear write index
        data_received = TRUE;                   // Inform caller
      }
    } else {                                    // No more bytes in the UART
      uart_empty = TRUE;                        // Force function exit
    }
  }
  return data_received;                         // Inform caller
}


//----------------------------------------------------------------------------
//
//  DESCRIPTION  Checks CRC8 of BCH data received from coder B
/// \return      TRUE if CRC8 is correct, FALSE otherwise
/// \remarks     -
//
//----------------------------------------------------------------------------
BOOLEAN8 Check_BCH(void)
{
   return (BOOLEAN8)(Crc8_Control_Rx(FALSE) == 0);
}

//----------------------------------------------------------------------------
//
//  DESCRIPTION Send telegram to coder B
/// \return     -
/// \remarks    -
//
//----------------------------------------------------------------------------
void Send_TLG_Coder_B(void)
{
   INT8U i;

   (void)Crc8_Control_Tx(TRUE);                    // Init CRC8 Register
   for (i = 0; i < DATA_FIELD_LENGTH; i++) {
     Crc8_Compute_Tx(TLG_Buffer[i]);               // Calculate CRC8
     (void)Uart0_PutC(TLG_Buffer[i]);              // Send telegram to coder B
   }
   TLG_Buffer[TLG_CRC8] = Crc8_Control_Tx(FALSE);  // Insert CRC8 value at the end
   (void)Uart0_PutC(TLG_Buffer[TLG_CRC8]);         // Send CRC8 to coder B
   Uart0_Start_Tx();                               // Start UART transmission
   BCH_Write_Index = 0;                            // Reset write index
}

//----------------------------------------------------------------------------
//
//  DESCRIPTION Send telegram + BCH to radio transceiver
/// \return     -
/// \remarks    Data is sent via SPI interface
//
//----------------------------------------------------------------------------
void Send_TLG_RTX(void)
{
  (void)Spi_Puts(TLG_Buffer, DATA_FIELD_LENGTH);   // Send data telegram
  (void)Spi_Puts(BCH_Buffer, BCH_FIELD_LENGTH);    // Send BCH code
}

//----------------------------------------------------------------------------
//
//  DESCRIPTION Get status of safety protected commands (SAFETY, directions,
//              joysticks, D2-D20)
/// \return     0 (CMD_OFF) if all commands are at standstill position \n
///             a number != 0 if at least one command is active.
/// \remarks    This function should be executed after the whole telegram has
///             been built, in order to check the data only after coder A
///             and coder B agreed upon their input data.
///             Commands values have following priority:
///             \n\n
///             CMD_SAFETY > CMD_D2D10 > CMD_DIR > CMD_D11D20 > CMD_ANALOG
///             \n\n
///             i.e. if SAFETY command is active, the return value will be
///             CMD_SAFETY regardless of the status of lower priority commands.
///             This specific priority order is needed by Stateflow to check
///             the commands at start up and to disable auto shut off.
//
//----------------------------------------------------------------------------
INT8U In_Commands (void)
{
    INT8U i, cmd;

    cmd = CMD_OFF;
    if (((TLG_Buffer[SAFETY] & MASK_TLG_SAF) != 0) &&           // SAFETY bit is 1
       (((Coder_A_Data[CODED_SAFETY] & MASK_SAF) != 0) ||       // SAFETY read by coder A is active
        ((Coder_B_Data[CODED_SAFETY] & MASK_SAF) != 0))) {      // SAFETY read by coder B is active
       cmd = CMD_SAFETY;                                        // SAFETY command active
    } else if (((TLG_Buffer[DIGITAL_1] & MASK_D2_D8) != 0) ||   // D2-D8 are active
               ((TLG_Buffer[DIGITAL_2] & MASK_D9_D10) != 0)) {  // D9-D10 are active
      cmd = CMD_D2D10;                                          // D2-D10 digital commands active
    } else if ((TLG_Buffer[DIRECTION_L] != 0) ||                // H directions read by coder A are active
               (TLG_Buffer[DIRECTION_H] != 0)) {                // L directions read by coder B are active
      cmd = CMD_DIR;                                            // Direction commands active
    } else if (((Digital_Get_Cmd(DIGITAL_D3) & MASK_D17_D20) != 0) || // D17-D20 are active
               ((TLG_Buffer[DIGITAL_2] & MASK_D11_D16) != 0) ){ // D11-D16 are active
        cmd = CMD_D11D20;                                       // D11-D20 digital commands active
    } else {                                                    //
      for (i = 0; i <= CODED_ANALOG_8; i++) {                   // Check joysticks
        if ((TLG_Buffer[ANALOG_8 + i] < VTAP_MIN) ||            // Joystick moved downward
            (TLG_Buffer[ANALOG_8 + i] > VTAP_MAX)) {            // Joystick moved upward
          cmd = CMD_ANALOG;                                     // Joystick active
        }
      }
    }
    return cmd;
}


//-----------------------------------------------------------------------------
//
//  DESCRIPTION Get status of START (D1) command
/// \return     TRUE if START is active, FALSE if START is released
/// \remarks    This function should be executed after the whole telegram has
///             been built, in order to check the data only after coder A
///             and coder B agreed upon their input data.
//
//-----------------------------------------------------------------------------
BOOLEAN8 In_Start(void)
{
  return (BOOLEAN8)((TLG_Buffer[DIGITAL_1] & MASK_D1) != 0);
}

