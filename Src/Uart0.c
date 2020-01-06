//============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/Uart0.c $
// $Revision: 8859 $
// $Date: 2010-10-29 11:51:20 +0200 (ven, 29 ott 2010) $
// $Author: micheleb $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief Uart 0 driver.
///
/// \file
///
/// \par COMMON RESOURCES:
/// Variable "U0_TX_Read_Index" is modified by function Uart0_Start_Tx() and
/// by Uart0_Tx_ISR() interrupt routine. Contention is avoided because:
/// - Uart0_Start_Tx() is called to start another transmission.
/// - Transmit interrupt disables itself at the end of each transmission.
/// - Uart0_Start_Tx() checks if a transmission is in progress before
///   modifying "U0_TX_Read_Index".
///
// CHANGES Removed trigraphs
//         Code inspection: multiple assignments, common resources, names, 
//         header, remarks
//
//============================================================================

#include  <ioavr.h>

#include "includes.h"
#include "Uart0.h"

/*--------------------------------- Definitions ------------------------------*/

#undef VAR_STATIC
#define VAR_STATIC static

#define U0_TX_BUFFER_SIZE    0x20   ///< Size of transmission buffer.
#define U0_RX_BUFFER_SIZE    0x20   ///< Size of reception buffer.

                          // BAUD
//#define UBRR0H_INIT 1   ///< 1200 bps
//#define UBRR0H_INIT 0   ///< 2400 bps
//#define UBRR0H_INIT 0   ///< 4800 bps
//#define UBRR0H_INIT 0   ///< 9600 bps
//#define UBRR0H_INIT 0   ///< 19200 bps
//#define UBRR0H_INIT 0   ///< 38400 bps
#define UBRR0H_INIT 0     ///< 100000 bps

//#define UBRR0L_INIT 159 ///< 1200 bps
//#define UBRR0L_INIT 207 ///< 2400 bps
//#define UBRR0L_INIT 103 ///< 4800 bps
//#define UBRR0L_INIT 51  ///< 9600 bps
//#define UBRR0L_INIT 25  ///< 19200 bps
//#define UBRR0L_INIT 12  ///< 38400 bps
#define UBRR0L_INIT 4     ///< 100000 bps

/*----------------------------------- Macros ----------------------------------*/

//! Transmission buffer is empty
#define U0_TX_BUFFER_EMPTY()      (U0_TX_Read_Index == U0_TX_Write_Index)

//! Transmission buffer is full
#define U0_TX_BUFFER_FULL()       (((INT8U)(U0_TX_Write_Index + 1) % U0_TX_BUFFER_SIZE) == U0_TX_Read_Index)

//! Disable transmission interrupt
#define U0_DISABLE_TX_INT()       (UCSR0B &= ~(1 << UDRIE0))

//! Transmission interrupt is enabled
#define U0_IS_TX_INT_ENABLED()    ((UCSR0B & (1 << UDRIE0)) != 0)

//! Reception buffer is empty
#define U0_RX_BUFFER_EMPTY()      (U0_RX_Read_Index == U0_RX_Write_Index)

//! Reception buffer is full
#define U0_RX_BUFFER_FULL()       (((INT8U)(U0_RX_Write_Index + 1) % U0_RX_BUFFER_SIZE) == U0_RX_Read_Index)

//! Enable reception interrupt
#define U0_ENABLE_RX_INT()        (UCSR0B |= (1 << RXCIE0))

//! Enable reception
#define U0_ENABLE_RX()            (UCSR0B |= (1 << RXEN0))

//! Enable transmission
#define U0_ENABLE_TX()            (UCSR0B |= (1 << TXEN0))

//! Enable transmission interrupt
#define U0_ENABLE_TX_INT()        (UCSR0B |= (1 << UDRIE0))

//! Transmission register is empty
#define U0_DATA_BUF_EMPTY()       ((UCSR0A & (1 << UDRE0)) != 0)

/*----------------------------------- Globals ---------------------------------*/

/*------------------------------------ Locals ---------------------------------*/

VAR_STATIC INT8U U0_RX_Buffer[U0_RX_BUFFER_SIZE];   ///< Reception buffer.
VAR_STATIC INT8U U0_TX_Buffer[U0_TX_BUFFER_SIZE];   ///< Transmission buffer.
VAR_STATIC INT8U U0_RX_Write_Index;                 ///< Reception buffer write index.
VAR_STATIC INT8U U0_RX_Read_Index;                  ///< Reception buffer read index.
VAR_STATIC INT8U U0_TX_Write_Index;                 ///< Transmission buffer write index.
VAR_STATIC INT8U U0_TX_Read_Index;                  ///< Transmission buffer read index.

/*---------------------------------- Prototypes -------------------------------*/

//-----------------------------------------------------------------------------
//
//  DESCRIPTION String length
/// \param      str Pointer to null termined string
/// \param      max_size Maximum string length
/// \return     number of bytes of string
/// \remarks    max_size prevents infinite loop, if str is not null terminated
//
//-----------------------------------------------------------------------------
static INT8U StrLen(const INT8U * str, INT8U max_size)
{
  const INT8U *s;
  INT8U i;

  s = str;
  for (i = 0; ((*s != '\0') && (i < max_size)); ++s) {
    i++;
  }
  return (INT8U)(s - str);
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Initialize UART 0
/// \remarks    -
//
//-----------------------------------------------------------------------------
void Uart0_Init(void)
{
  PORTE |= 0x03;                // Enable pull-up to set line in idle state
  UCSR0A = (INT8U) 0x02;        // Double speed
  UCSR0C = (INT8U) 0x06;        // 8 data bit - 1 Stop bit - No parity
  UBRR0H = UBRR0H_INIT;         // Baud-rate high byte
  UBRR0L = UBRR0L_INIT;         // Baud-rate low byte
  U0_RX_Write_Index = 0;        // Reset buffer indexes
  U0_RX_Read_Index = 0;         //
  U0_TX_Write_Index = 0;        //
  U0_TX_Read_Index = 0;         //
  U0_ENABLE_TX();               // Enable TX
  U0_ENABLE_RX();               // Enable RX
  U0_ENABLE_RX_INT();           // Enable RX interrupt
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Gets one character from the receiver buffer, if any.
/// \param      p_ch pointer to destination character.
/// \return     TRUE if buffer contains at least a byte, FALSE otherwise.
/// \remarks    -
//
//-----------------------------------------------------------------------------
BOOLEAN8 Uart0_GetC(INT8U * p_ch)
{
  BOOLEAN8 ret;

  if (U0_RX_BUFFER_EMPTY()) {                   // RX buffer empty
    ret = FALSE;
  } else {                                      // Rx buffer not empty
    *p_ch = U0_RX_Buffer[U0_RX_Read_Index++];   // Return next read data from RX buffer
    U0_RX_Read_Index %= U0_RX_BUFFER_SIZE;      // Read index wrap around
    ret = TRUE;
  }
  return ret;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Puts a character into transmission buffer, if not full.
/// \param      ch Character to be sent
/// \return     TRUE if transmission buffer was free, FALSE otherwise
/// \remarks    Buffer is full when there is only 1 byte free
//
//-----------------------------------------------------------------------------
BOOLEAN8 Uart0_PutC(INT8U ch)
{
  BOOLEAN8 not_full = FALSE;

  if ( !U0_TX_BUFFER_FULL() ) {              // Transmit buffer not full
    U0_TX_Buffer[U0_TX_Write_Index++] = ch;  // Put new character into buffer
    U0_TX_Write_Index %= U0_TX_BUFFER_SIZE;  // Update write index
    not_full = TRUE;                         // Buffer not full
  }
  return not_full;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Put a string into transmission buffer, if not full.
/// \param      psz_string  Pointer to null terminated string.
/// \return     TRUE if the whole string fits into buffer, FALSE otherwise
/// \remarks    String must be null terminated.
//
//-----------------------------------------------------------------------------
BOOLEAN8 Uart0_PutS(const INT8U * psz_string)
{
  BOOLEAN8 copied = TRUE;                           //
  INT8U tmp_index = U0_TX_Write_Index;              // Temporary write index

  while ((*psz_string != '\0') &&                   // End of string not reached
         (copied == TRUE)) {                        // Character copied
    if (((INT8U)(tmp_index + 1) % U0_TX_BUFFER_SIZE) != U0_TX_Read_Index) {
                                                    // Tx buffer not full
      U0_TX_Buffer[tmp_index++] = *(psz_string++);  // Copy character into buffer
      tmp_index %= U0_TX_BUFFER_SIZE;               // Wrap around temporary index
      copied = TRUE;                                // One char copied
    } else {                                        // No character copied
      copied = FALSE;                               // String not copied
    }
  }
  if (copied == TRUE) {                             // Character copied
    U0_TX_Write_Index = tmp_index;                  // Update actual write index
  }
  return copied;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Gets a line from receiver buffer, if any
/// \param      psz_line pointer to destination string.
/// \param      size string length
/// \return     TRUE if a CR terminated string was found, FALSE otherwise.
/// \remarks    Caller must take care of clearing string pointed to by psz_line
//
//-----------------------------------------------------------------------------
BOOLEAN8 Uart0_GetS(INT8U * psz_line, INT8U size)
{
  INT8U ch, len;
  INT8U *psz_end_line = psz_line;
  BOOLEAN8 found = FALSE;

  if (Uart0_GetC(&ch)) {
    if (ch != '\n') {                   // Line feed always discarded
      if (ch != '\r') {                 // CR not found
        len = StrLen(psz_line,size);    // Compute length of string
        if ((len + 1) < size) {         // Destination has 2+ spare bytes
          psz_end_line += len;          // Append char at the end of the string
          *psz_end_line++ = ch;
          *psz_end_line = 0;            // Null-terminated string
        } else {                        // Destination has no spare bytes
          found = TRUE;                 // Consider string ended
        }
      } else {                          // CR found
        found = TRUE;                   // String ended
      }
    }
  }
  return found;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Start UART 0 transmission.
/// \remarks    First byte is explicitly transmitted inside this function,
///             transmit interrupt will take care of the transmission of
///             remaining data.
///             'While' loop waits for UART transmit register to be free and
///             is blocking. With a baud rate of 100 Kbps it exits in 100 us
///             at worst.
//
//-----------------------------------------------------------------------------
void Uart0_Start_Tx(void)
{
  if ((!U0_IS_TX_INT_ENABLED()) &&              // Transmit interrupt not enabled
      (!U0_TX_BUFFER_EMPTY())) {                // Transmit buffer not empty
    while ((!U0_DATA_BUF_EMPTY())) {            // Wait until TX register is free
      ;
    }
    UDR0 = U0_TX_Buffer[U0_TX_Read_Index++];    // Copy a byte from buffer to TX register
    U0_TX_Read_Index %= U0_TX_BUFFER_SIZE;      // Wrap around read index
    if ((!U0_TX_BUFFER_EMPTY())) {              // Transmit buffer not empty
      U0_ENABLE_TX_INT();                       // Enable transmit interrupt
    }
  }
}

//-----------------------------------------------------------------------------
//
// DESCRIPTION UART 0 transmission interrupt routine
/// \remarks   After last byte has been sent, transmission interrupt is disabled
//
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Uart0_Tx_ISR() not referenced"
#pragma vector=USART0_UDRE_vect
static __interrupt void Uart0_Tx_ISR (void)
{
  if ((!U0_TX_BUFFER_EMPTY())) {                // Transmit buffer not empty
     UDR0 = U0_TX_Buffer[U0_TX_Read_Index++];   // Copy a byte from buffer to TX register
     U0_TX_Read_Index %= U0_TX_BUFFER_SIZE;     // Wrap around read index
  } else {                                      // Transmit buffer empty
     U0_DISABLE_TX_INT();                       // Disable transmit interrupt
  }
}

//-----------------------------------------------------------------------------
//
// DESCRIPTION  UART 0 reception interrupt routine
/// \remarks    -
//
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Uart0_Rx_ISR() not referenced"
#pragma vector=USART0_RXC_vect
static __interrupt void Uart0_Rx_ISR(void)
{
  if ((!U0_RX_BUFFER_FULL())) {                  // RX buffer not full
    U0_RX_Buffer[U0_RX_Write_Index++] = UDR0;    // Save received byte into buffer
    U0_RX_Write_Index %= U0_RX_BUFFER_SIZE;      // Update buffer write index
  }
}
