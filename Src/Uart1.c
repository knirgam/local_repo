//============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/Uart1.c $
// $Revision: 9480 $
// $Date: 2011-01-13 16:54:01 +0100 (gio, 13 gen 2011) $
// $Author: micheleb $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief Uart 1 driver.
///
/// \file
///
// CHANGES Corrected problem of receive interrupt never exiting.
//
//============================================================================

#include  <ioavr.h>

#include "includes.h"
#include "Uart1.h"

/*--------------------------------- Definitions ------------------------------*/

#undef VAR_STATIC
#define VAR_STATIC static

#define U1_TX_BUFFER_SIZE    0x20   ///< Size of transmission buffer.
#define U1_RX_BUFFER_SIZE    0x20   ///< Size of reception buffer.

                          // BAUD
//#define UBRR1H_INIT 1   ///< 1200 bps
//#define UBRR1H_INIT 0   ///< 2400 bps
//#define UBRR1H_INIT 0   ///< 4800 bps
#define UBRR1H_INIT 0     ///< 9600 bps
//#define UBRR1H_INIT 0   ///< 19200 bps
//#define UBRR1H_INIT 0   ///< 38400 bps
//#define UBRR1H_INIT 0   ///< 100000 bps

//#define UBRR1L_INIT 159 ///< 1200 bps
//#define UBRR1L_INIT 207 ///< 2400 bps
//#define UBRR1L_INIT 103 ///< 4800 bps
#define UBRR1L_INIT 51    ///< 9600 bps
//#define UBRR1L_INIT 25  ///< 19200 bps
//#define UBRR1L_INIT 12  ///< 38400 bps
//#define UBRR1L_INIT 4   ///< 100000 bps

/*---------------------------------- Macros ---------------------------------*/

//! Transmission buffer is empty
#define U1_TX_BUFFER_EMPTY()      (U1_TX_Read_Index == U1_TX_Write_Index)

//! Transmission buffer is full
#define U1_TX_BUFFER_FULL()       (((INT8U)(U1_TX_Write_Index + 1) % U1_TX_BUFFER_SIZE) == U1_TX_Read_Index)

//! Reception buffer is empty
#define U1_RX_BUFFER_EMPTY()      (U1_RX_Read_Index == U1_RX_Write_Index)

//! Reception buffer is full
#define U1_RX_BUFFER_FULL()       (((INT8U)(U1_RX_Write_Index + 1) % U1_RX_BUFFER_SIZE) == U1_RX_Read_Index)

//! Enable reception interrupt
#define U1_ENABLE_RX_INT()        (UCSR1B |= (1 << RXCIE1))

//! Enable reception
#define U1_ENABLE_RX()            (UCSR1B |= (1 << RXEN1))

//! Enable transmission
#define U1_ENABLE_TX()            (UCSR1B |= (1 << TXEN1))

//! Transmission register is empty
#define U1_READY()                ((UCSR1A & (1 << UDRE1)) != 0)

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC INT8U U1_RX_Buffer[U1_RX_BUFFER_SIZE];   ///< Reception buffer.
VAR_STATIC INT8U U1_TX_Buffer[U1_TX_BUFFER_SIZE];   ///< Transmission buffer.
VAR_STATIC INT8U U1_RX_Write_Index;                 ///< Reception buffer write index.
VAR_STATIC INT8U U1_RX_Read_Index;                  ///< Reception buffer read index.
VAR_STATIC INT8U U1_TX_Write_Index;                 ///< Transmission buffer write index.
VAR_STATIC INT8U U1_TX_Read_Index;                  ///< Transmission buffer read index.

/*--------------------------------- Prototypes -------------------------------*/


//-----------------------------------------------------------------------------
//
//  DESCRIPTION  Initialize UART 1
/// \remarks     -
//
//-----------------------------------------------------------------------------
void Uart1_Init(void)
{
  PORTD |= 0x0C;            // Enable pull-ups to set line in idle state
  UCSR1A = (INT8U) 0x02;    // Double speed
  UCSR1C = (INT8U) 0x06;    // 8 data bit - 1 Stop bit - No parity
  UBRR1H = UBRR1H_INIT;     // Baud-rate high byte
  UBRR1L = UBRR1L_INIT;     // Baud-rate low byte
  U1_RX_Write_Index = 0;    // Reset buffer indexes
  U1_RX_Read_Index = 0;     //
  U1_TX_Write_Index = 0;    //
  U1_TX_Read_Index = 0;     //
  U1_ENABLE_TX();           // Enable TX
  U1_ENABLE_RX();           // Enable RX
  U1_ENABLE_RX_INT();       // Enable RX interrupt
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION  Gets one character from the receiver buffer, if any.
/// \param       p_ch Pointer to destination character.
/// \return      TRUE if buffer contains at least a byte, FALSE otherwise.
/// \remarks     -
//
//-----------------------------------------------------------------------------
BOOLEAN8 Uart1_GetC(INT8U * p_ch)
{
  BOOLEAN8 found;

  if (U1_RX_BUFFER_EMPTY()) {                     // RX buffer empty
    found = FALSE;
  } else {                                        // Rx buffer not empty
    *p_ch = U1_RX_Buffer[U1_RX_Read_Index++];     // Get byte from RX buffer
    U1_RX_Read_Index %= U1_RX_BUFFER_SIZE;        // Read index wrap around
    found = TRUE;
  }
  return found;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION  Puts a character into transmission buffer, if not full.
/// \param       ch Character to be sent
/// \return      TRUE if transmission buffer was free, FALSE otherwise
/// \remarks     Buffer is full when there is only 1 byte free
//
//-----------------------------------------------------------------------------
BOOLEAN8 Uart1_PutC(INT8U ch)
{
  BOOLEAN8 not_full = FALSE;

  if ( !U1_TX_BUFFER_FULL() ) {             // Transmit buffer not full
    U1_TX_Buffer[U1_TX_Write_Index++] = ch; // Put new character into buffer
    U1_TX_Write_Index %= U1_TX_BUFFER_SIZE; // Update write index
    not_full = TRUE;                        // Buffer not full
  }
  return not_full;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Put a string into transmission buffer, if not full.
/// \param      psz_string Pointer to null terminated string.
/// \return     TRUE if the whole string fits into buffer, FALSE otherwise
/// \remarks    String must be null terminated.
//
//-----------------------------------------------------------------------------
BOOLEAN8 Uart1_PutS(const INT8U * psz_string)
{
  BOOLEAN8 copied = TRUE;                           //
  INT8U tmp_index = U1_TX_Write_Index;              // Temporary write index

  while ((*psz_string != '\0') &&                   // End of string not reached
         (copied == TRUE)) {                        // Character copied
    if (((INT8U)(tmp_index + 1) % U1_TX_BUFFER_SIZE) != U1_TX_Read_Index) {
                                                    // Tx buffer not full
      U1_TX_Buffer[tmp_index++] = *(psz_string++);  // Copy character into buffer
      tmp_index %= U1_TX_BUFFER_SIZE;               // Wrap around temporary index
      copied = TRUE;                                // One char copied
    } else {                                        // No character copied
      copied = FALSE;                               // String not copied
    }
  }
  if (copied == TRUE) {                             // Character copied
    U1_TX_Write_Index = tmp_index;                  // Update actual write index
  }
  return copied;
}


//-----------------------------------------------------------------------------
//
//  DESCRIPTION UART 1 manager
/// \remarks    Retrieves bytes from tranmsission buffer and sends them to
///             UART 1 transmission register.
//
//-----------------------------------------------------------------------------
void Uart1_Manager(void)
{

  Ctrl_Flow_Cnt += CALLED_UART1;

  if (U1_READY() &&                             // UART 1 ready and ...
     (!U1_TX_BUFFER_EMPTY())) {                 // ... transmitt buffer not empty
    UDR1 = U1_TX_Buffer[U1_TX_Read_Index++];    // Transmit next byte
    U1_TX_Read_Index %= U1_TX_BUFFER_SIZE;      // Wrap around read index
  }

  Ctrl_Flow_Cnt_Inv -= CALLED_UART1;

}

//-----------------------------------------------------------------------------
//
// DESCRIPTION UART 1 reception interrupt routine
/// \remarks   UDR1 should be read in order to clear interrupt flag.
///            Failing to do so will result in the CPU locked in the interrupt
///            routine.
//
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Uart1_Rx_ISR() not referenced"
#pragma vector=USART1_RXC_vect
static __interrupt void Uart1_Rx_ISR(void)
{ INT8U temp;

  temp = UDR1;                                  // Read received byte
  if ((!U1_RX_BUFFER_FULL())) {                 // RX buffer not full
    U1_RX_Buffer[U1_RX_Write_Index++] = temp;   // Save received byte into buffer
    U1_RX_Write_Index %= U1_RX_BUFFER_SIZE;     // Update buffer write index
  }
}
