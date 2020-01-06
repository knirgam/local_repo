//============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/Spi.c $
// $Revision: 9742 $
// $Date: 2011-02-09 10:05:27 +0100 (mer, 09 feb 2011) $
// $Author: micheleb $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief SPI driver module
///
/// \file
///
// CHANGES doxygen
//
//============================================================================

#include  <ioavr.h>

#include "includes.h"
#include "tmwtypes.h"
#include "Digital.h"
#include "tpool.h"
#include "Spi.h"

#include "Logic.h"

/*--------------------------------- Definitions ------------------------------*/

#undef  VAR_STATIC
#define VAR_STATIC static

#define TSTAMP_MASK     (INT8U)(0x3F)           //!< Timestamp mask
#define TSTAMP_INV_MASK (INT8U)(0x40)           //!< Inverted timestamp mask

#define SPI_BUFFER_SIZE ((INT8U)TLG_SIZE * 2)   //!< Length of SPI buffer
#define TLG_SIZE        (INT8U)27               //!< Telegram length

#define SPI_SS    (INT8U)0x01   //!< Mask for Slave Select input pin
#define SPI_CLK   (INT8U)0x02   //!< Mask for Clock signal input pin
#define SPI_MOSI  (INT8U)0x04   //!< Mask for MOSI input pin
#define SPI_MISO  (INT8U)0x08   //!< Mask for MISO input pin

/*------------------------------------ Macros --------------------------------*/

//! Turn on internal green LED
#define LED_GREEN_ON()              (PORTA &= ~0x01)

//! Turn off internal green LED
#define LED_GREEN_OFF()             (PORTA |= 0x01)

//! Disable SPI interrupt.
#define SPI_DISABLE_INTERRUPT()     (SPCR &= ~(1<<SPIE))

//! Enable SPI interrupt.
#define SPI_ENABLE_INTERRUPT()      (SPCR |= (1<<SPIE))

//! Enable SPI
#define SPI_ENABLE()                (SPCR |= (1<<SPE))

//! Disable SPI
#define SPI_DISABLE()               (SPCR &= ~(1<<SPE))

//! Enable external interrupt on INT4 - SS
#define ENABLE_SS_INT()             (EIMSK |= (1 << INT4))

//! Clear external interrupt flag on INT4 - SS
#define CLEAR_SS_INT()              (EIFR |= (1 << INTF4))

/*----------------------------------- Globals --------------------------------*/

/*------------------------------------ Locals --------------------------------*/

VAR_STATIC INT8U Spi_Buffer[SPI_BUFFER_SIZE]; //!< SPI transmission buffer.
VAR_STATIC INT8U Spi_Write_Index;             //!< SPI buffer write index
VAR_STATIC INT8U Spi_Read_Index;              //!< SPI buffer read index
VAR_STATIC BOOLEAN8 Flg_RTX_Request;          //!< Telegram request from RTX

__no_init VAR_STATIC INT8U Timestamp     @ "SAFETY_RAM";    //!< Telegram timestamp
__no_init VAR_STATIC INT8U Timestamp_Inv @ "SAFETY_RAM_REV";//!< Telegram timestamp, inverted

__no_init VAR_STATIC INT8U TLG_Mux       @ "SAFETY_RAM";    //!< Telegram multiplexer
__no_init VAR_STATIC INT8U TLG_Mux_Inv   @ "SAFETY_RAM_REV";//!< Telegram multiplexer, inverted

/*---------------------------------- Prototypes -------------------------------*/

/*------------------------------- Inline Function -----------------------------*/

//! Clear pending SPI interrupt.
#pragma inline=forced
static void SPI_CLEAR_INTERRUPT(void)
{
  SPSR |= (1 << SPIF);
  SPDR = SPDR;
}

//!< Select falling edge on INT4 - SS
#pragma inline=forced
static void FALLING_EDGE_SS(void)
{
  EICRB &= ~(0x03);
  EICRB |= 0x02;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION  SPI Initialization
/// \remarks     -
//
//-----------------------------------------------------------------------------
void Spi_Init(void)
{
  INT8U j;

  PORTB &= ~(SPI_MOSI + SPI_CLK + SPI_SS);  // No pull-up resistors
  DDRB &= ~(SPI_MOSI + SPI_CLK + SPI_SS);   // PB0, PB1, PB2 as inputs
  PORTB |= SPI_MISO;                        // Pull-up on PB3 (RTX_TLG)
  DDRB |= SPI_MISO;                         // PB3 (RTX_TLG) as output
  DDRE &= ~(0x10);                          // PE4 as input
  SPCR = 0x04;                              // CPOL-CPHA -> SPI mode = 1
                                            // (Setup on rising edge,
                                            // Sampling on falling edge)
  FALLING_EDGE_SS();                        // Select falling edge interrupt on SS
  CLEAR_SS_INT();                           // Clear pending interrupt on SS
  ENABLE_SS_INT();                          // Enable external interrupt on SS
  for (j = 0; j < SPI_BUFFER_SIZE; j++) {   // Clear transmission buffer
    Spi_Buffer[j] = 0;
  }
  Spi_Read_Index = 0;                       // Reset read index
  Spi_Write_Index = 0;                      // Reset write index
  Flg_RTX_Request = FALSE;                  // Reset telegram request from RTX
  TLG_Mux = 0;                              // Clear telegram multiplexer
  TLG_Mux_Inv = 0xFF;                       //
  Timestamp = 0;                            // Clear telegram timestamp
  Timestamp_Inv = 0xFF;                     //
  TPool_Start(TMR_TSTAMP, TMR_50_ms);       // Start timer for timestamp update
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION  Writes data into SPI transmission buffer.
/// \param       ps_data pointer to source data
/// \param       len number of bytes to be written
/// \return      TRUE if all bytes have been copied, FALSE otherwise
/// \remarks     Transmission buffer is circular.
//
//-----------------------------------------------------------------------------
BOOLEAN8 Spi_Puts(const INT8U * ps_data, INT8U len)
{
  BOOLEAN8 data_copied = FALSE;
  INT8U j, end_loop;

  if (Spi_Write_Index >= TLG_SIZE) {
    end_loop = 0;
  } else {
    end_loop = TLG_SIZE;
  }

  for (j = 0; (j < len) && (Spi_Write_Index != end_loop); j++) {
    Spi_Buffer[Spi_Write_Index++] = *ps_data++; // Copy data into SPI transmission buffer
    if (Spi_Write_Index == SPI_BUFFER_SIZE) {
      Spi_Write_Index = 0;                      // Wrap write index
    }
  }
  if (j == len) {
    data_copied = TRUE;                         // Data has been copied successfully
  }

  return data_copied;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION  Get current value for timestamp
/// \return      timestamp value
/// \remarks     -
//
//-----------------------------------------------------------------------------
INT8U Get_Timestamp(void)
{
  return Timestamp;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION  Get current value for telegram multiplexer
/// \return      Telegram multiplexer value
/// \remarks     -
//
//-----------------------------------------------------------------------------
INT8U Get_TLG_Mux(void)
{
  return TLG_Mux;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Checks timely requests from RTX and updates timestamp
/// \remarks    -
//
//-----------------------------------------------------------------------------
void RTX_Timeout(void)
{ INT8U delta = 1;

  Ctrl_Flow_Cnt += CALLED_RTX;

  if (Flg_RTX_Request == TRUE) {            // RTX has requested a new telegram
    broadcast_Logic_Ev_Tlg();               // Send telegram event to Stateflow
    Flg_RTX_Request = FALSE;                // Clear flag
  }

  if ((Timestamp ^ Timestamp_Inv) != 0xFF) {// Critical variable error
    Shut_Down_TU();                         // Shut down transmitter
    Shut_Down_TU();                         // make sure the flip flop toggles
  }
  if (TPool_End(TMR_TSTAMP)) {              // Timer expired
    TPool_Start(TMR_TSTAMP, TMR_50_ms);     // Restart timer
    Timestamp += delta;                     // Update timestamp
    Timestamp &= TSTAMP_MASK;               // Wrap around
    Timestamp_Inv -= delta;                 // Update inverted timestamp
    Timestamp_Inv |= TSTAMP_INV_MASK;       // Wrap around
  }

  Ctrl_Flow_Cnt_Inv -= CALLED_RTX;
}

//----------------------------------------------------------------------------
//
//  DESCRIPTION Slave Select input interrupt
/// \return     -
/// \remarks    Interrupt is triggered on falling edge of SS (slave select).
///             Flushes previous transmission and starts a new request.
//
//----------------------------------------------------------------------------
//lint --e{528} suppress "SS_ISR() not referenced"
# pragma vector=INT4_vect
static __interrupt void SS_ISR(void)
{
  if (Spi_Write_Index >= TLG_SIZE) {        // Write index passed buffer half
    Spi_Read_Index = 0;                     // Send first half of buffer
  } else {                                  //
    Spi_Read_Index = TLG_SIZE;              // Send second half of buffer
  }
  SPI_CLEAR_INTERRUPT();                    // Clear pending interrupt
  SPDR = Spi_Buffer[Spi_Read_Index++];      // Put first byte into SPI buffer
  SPI_ENABLE();                             // Enable SPI transmission
  SPI_ENABLE_INTERRUPT();                   // Enable SPI interrupt
  if ((TLG_Mux ^ TLG_Mux_Inv) == 0xFF) {    // Check critical variable
    TLG_Mux ^= 1;                           // Update telegram multiplexer
    TLG_Mux_Inv ^= 1;
  } else {
    Shut_Down_TU();                         // Called twice to be sure the
    Shut_Down_TU();                         // flip flop toggles its status
  }
  LED_GREEN_ON();                           // Turn on green LED
}

//-----------------------------------------------------------------------------
//
// DESCRIPTION  SPI tranmsmision interrupt
/// \return     -
/// \remarks    Copy another byte from transmission buffer into SPI register.
///             Transmission ends when all telegram bytes have been sent.
//
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Spi_Tx_ISR() not referenced"
#pragma vector=SPI_STC_vect
static __interrupt void Spi_Tx_ISR(void)
{
  if ((Spi_Read_Index == TLG_SIZE) ||
      (Spi_Read_Index == SPI_BUFFER_SIZE)) {    // End of telegram
    LED_GREEN_OFF();                            // Turn off green LED
    SPI_DISABLE();                              // Disable SPI
    SPI_DISABLE_INTERRUPT();                    // Disable SPI interrupt
    Flg_RTX_Request = TRUE;                     // RTX requested a telegram
  } else {                                      // Telegram not ended
    SPDR = Spi_Buffer[Spi_Read_Index++];        // Copy byte into SPI register
  }
}
