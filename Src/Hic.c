//============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/Hic.c $
// $Revision: 9742 $
// $Date: 2011-02-09 10:05:27 +0100 (mer, 09 feb 2011) $
// $Author: lorenzo $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief HIC communication management module
///
/// \file
///
/// \par COMMON RESOURCES: CRC
/// CRC module is shared by HIC manager, UID manager and Code module.\n
/// See CRC module for explanation of contention management.
///
// CHANGES doxygen
//
//============================================================================

#include <ioavr.h>

#include "includes.h"
#include "Crc.h"
#include "Uart1.h"
#include "version.h"
#include "TPool.h"
#include "Hic.h"

/*--------------------------------- Definitions ------------------------------*/

#undef  VAR_STATIC
#define VAR_STATIC static

#define MESSAGE_SIZE    16                  ///< Size of HIC message string
#define HIC_BUF_SIZE    12                  ///< Size of HIC data without header
#define HIC_CRC         (HIC_BUF_SIZE - 2)  ///< Position of CRC inside HIC data
#define HIC_HEADER      0xCC                ///< Header of data frame from HIC
#define STRING_HEADER   0xCF                ///< Header of generic message to HIC
#define DIAG_HEADER     0xCA                ///< Header of diagnostic message to HIC
#define AUX_MASK        0xF0                ///< Mask for auxiliary commands (D21-D24)
#define TIMP_MASK       0x01                ///< Mask for TIMP bit

/*----------------------------------- Macros ---------------------------------*/

/*------------------------------------ Enums ---------------------------------*/

enum {
    GET_HEADER,
    GET_DATA,
    VERIFY_DATA
};

/*------------------------------------ Const ---------------------------------*/

/*----------------------------------- Globals --------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC INT8U Hic_Message[MESSAGE_SIZE]; ///< Message to be sent to HIC
VAR_STATIC INT8U Hic_Buffer[HIC_BUF_SIZE];  ///< Data received from HIC
VAR_STATIC INT8U Aux_Cmd[AUX_CMD_NUM];      ///< Auxiliary commands
VAR_STATIC INT8U Timp;                      ///< TIMP bit

/*---------------------------------- Prototypes -------------------------------*/

static INT8U Hex2Ascii (INT8U); //!< Converts a hex value into an ASCII character
static INT8U Ascii2Hex (INT8U); //!< Converts an ASCII character into a hex value
static void Reset_Aux(void);    //!< Reset all Auxiliary commands status

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Initialization of hic communiation module.
/// \return     -
/// \remarks    Firmware revision string is read here and sent to UART.
//
//-----------------------------------------------------------------------------
void Hic_Init(void)
{
  Reset_Aux();                              // Reset all commands
  Timp = 0;                                 // Default value for TIMP is 0
  Version_Read(Hic_Message, MESSAGE_SIZE);  // Read FW version string
  (void)Uart1_PutC(STRING_HEADER);          // Send STRING header to HIC
  (void)Uart1_PutS(Hic_Message);            // Send message to HIC
  TPool_Start(TMR_HIC, TMR_2_s);            // Start HIC communication time-out
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Hic communication manager
/// \return     -
/// \remarks    -
//
//-----------------------------------------------------------------------------
void Hic_Manager(void)
{
  static INT8U hic_status = GET_HEADER;
  static INT8U hic_write_index = 0;
  INT8U i, ch;

  Ctrl_Flow_Cnt += CALLED_HIC;

  switch (hic_status) {

    case GET_HEADER:
      if (Uart1_GetC(&ch) == TRUE) {            // New byte received by UART1
        if (ch == HIC_HEADER) {                 // Header received, start data reception
          (void)Crc8_Control_Rx_Hic(TRUE);      // Initialize CRC
          Crc8_Compute_Rx_Hic(ch);              // Calculate CRC on header too
          hic_write_index = 0;
          hic_status = GET_DATA;
        }
      }
      break;

    case GET_DATA:
      if (Uart1_GetC(&ch) == TRUE) {              // New byte received by UART1
        if (ch == HIC_HEADER) {                   // Header received again, re-start data reception
          (void)Crc8_Control_Rx_Hic(TRUE);        // Initialize CRC
          Crc8_Compute_Rx_Hic(ch);                // Calculate CRC on header too
          hic_write_index = 0;
        } else {                                  // Data bytes
          if (hic_write_index < HIC_CRC) {
            Crc8_Compute_Rx_Hic(ch);              // Calculate CRC on received data
          }
          Hic_Buffer[hic_write_index++] = ch;     // Store received byte into buffer
          if (hic_write_index == HIC_BUF_SIZE) {  // End of reception
            hic_status = VERIFY_DATA;             // Go to data verification
          }
        }
      }
      break;

    case VERIFY_DATA:                             // Compare CRC
      if (Crc8_Control_Rx_Hic(FALSE) ==
          ((INT8U)(Ascii2Hex(Hic_Buffer[HIC_CRC]) << 4) +
           (INT8U)Ascii2Hex(Hic_Buffer[HIC_CRC + 1]))) {
        hic_write_index = 0;
        for (i = 0; i < AUX_CMD_NUM; i++) {       // Decode data and update Aux commands buffer
           Aux_Cmd[i] = (INT8U)(Ascii2Hex(Hic_Buffer[hic_write_index]) << 4) +
                              (INT8U)Ascii2Hex(Hic_Buffer[hic_write_index + 1]);
           hic_write_index += 2;
        }
        Timp = Aux_Cmd[AUX_CMD_1] & TIMP_MASK;    // Update TIMP selection
        Aux_Cmd[AUX_CMD_1] &= AUX_MASK;           // Mask less significant nibble
        TPool_Start(TMR_HIC, TMR_2_s);     // Re-start timer for communication time-out
      }
      hic_status = GET_HEADER;                    // Get next data packet
      break;

    default:
      hic_status = GET_HEADER;                    // Get next data packet
      break;
  }

  if (TPool_End(TMR_HIC)) {                // Time-out elapsed
    Reset_Aux();                                  // Reset auxiliary commands
  }

  Ctrl_Flow_Cnt_Inv -= CALLED_HIC;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Reset Aux commands buffer
/// \return     -
/// \remarks    -
//
//-----------------------------------------------------------------------------
static void Reset_Aux(void)
{
  INT8U i;

  for (i = 0; i <  AUX_CMD_NUM; i++) {
    Aux_Cmd[i] = 0;
  }
}

//----------------------------------------------------------------------------
//
//  DESCRIPTION  Function that converts an ASCII char into hex value
/// \param       Byte (ASCII coded) to be converted
/// \return      Hex value of ASCII character
/// \remarks     -
//
//----------------------------------------------------------------------------
static INT8U Ascii2Hex (INT8U ch)
{
  INT8U hex_value;

  if ((ch >= '0') && (ch <= '9')) {         // Input between '0' and '9'
    hex_value = ch - '0';                   // Sub ASCII value for '0' to input
  } else if ((ch >= 'A') && (ch <= 'F')) {  // Input between 'A' and 'F'
    hex_value = (ch + 10) - 'A';            // Sub ASCII value for 'A' to input
  } else {
    hex_value = 0;
  }
  return hex_value;
}

//----------------------------------------------------------------------------
//
//  DESCRIPTION  Function that converts a hex value into ASCI char
/// \param       Byte (0-15) to be converted
/// \return      ASCII character corresponding to Hex value
/// \remarks     -
//----------------------------------------------------------------------------
static INT8U Hex2Ascii (INT8U hex_value)
{
  INT8U ch;

  hex_value &= 0x0F;                  // Mask with LS nibble
  if (hex_value < 10) {               // Input < 10
     ch = hex_value + '0';            // Add ASCII value for '0' to input
  } else if (hex_value < 16) {        // Input >= 10
     ch = (hex_value - 10) + 'A';     // Add ASCII value for 'A' to input
  } else {
    ch = '0';
  }
  return ch;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Get one byte of aux input commands
/// \param      index auxiliary command buffer index
/// \return     digital auxiliary commands byte, specified by index
/// \remarks    -
//
//-----------------------------------------------------------------------------
INT8U Hic_Get_Cmd(INT8U index)
{
  INT8U ret = 0;

  if (index < AUX_CMD_NUM) {   // Array bounds check
    ret = Aux_Cmd[index];
  }
  return ret;
}

//-----------------------------------------------------------------------------
///
//  DESCRIPTION Get TIMP bit
/// \return     Timp selection: 0 or 1
/// \remarks    -
//
//-----------------------------------------------------------------------------
INT8U Timp_Read(void)
{
  return Timp;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Send diagnostic codes to HIC by means of UART 1
/// \return     TRUE if transmission is allowed, FALSE otherwise
/// \remarks    -
//
//-----------------------------------------------------------------------------
BOOLEAN8 Send_Diagnostic(INT8U failure_code)
{
  BOOLEAN8 sent;

  (void)Crc8_Control_Tx_Hic(TRUE);          // Init CRC calculation

  //
  // Put Diagnostic header
  //
  Hic_Message[0] = DIAG_HEADER;
  Crc8_Compute_Tx_Hic(Hic_Message[0]);

  //
  // Insert ASCII-coded failure code
  //
  Hic_Message[1] = Hex2Ascii((INT8U)((INT8U)(failure_code & 0xF0) >> 4));
  Crc8_Compute_Tx_Hic(Hic_Message[1]);

  Hic_Message[2] = Hex2Ascii((INT8U)(failure_code & 0x0F));
  Crc8_Compute_Tx_Hic(Hic_Message[2]);

  //
  // Compute and append CRC
  //
  Hic_Message[3] = Hex2Ascii((INT8U)((INT8U)(Crc8_Control_Tx_Hic(FALSE) & 0xF0) >> 4));
  Hic_Message[4] = Hex2Ascii((INT8U)(Crc8_Control_Tx_Hic(FALSE) & 0x0F));

  //
  // Terminate string
  //
  Hic_Message[5] = '\0';

  //
  // Send to UART1
  //
#pragma diag_suppress=Pm038     // Bypass MISRA pointer use inhibition
  sent = Uart1_PutS((INT8U*)Hic_Message);
#pragma diag_error=Pm038
  return sent;
}
