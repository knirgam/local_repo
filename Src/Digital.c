//============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/Digital.c $
// $Revision: 9861 $
// $Date: 2011-03-08 14:59:06 +0100 (mar, 08 mar 2011) $
// $Author: lorenzo $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief Digital inputs driver.
///
/// \file
///          Digital commands are multiplexed on a single input port (port C)
///          of ATmega64, these are:
///
///              H direction inputs \n
///              L direction inputs \n
///              D1 - D8 digital inputs \n
///              D9 - D16 digital inputs \n
///
///          Selection of multiplexer input is made by means of following
///          outputs:
///
///              A_SEL_1 selects D1 - D8 digital inputs \n
///              A_SEL_2 selects D9 - D16 digital inputs \n
///              A_SEL_3 selects H direction inputs \n
///              A_SEL_4 selects L direction inputs \n
///              A_SEL_5 selects D17 - D20 digital inputs \n
///
///          Handshake between coder A and coder B is managed by BUS[3-4]
///          signals (BUS[1-2] signals are for communication):
///
///              BUS1 = serial communication receive input \n
///              BUS2 = serial communication transmit output \n
///              BUS3 = ready signal from coder B \n
///              BUS4 = trigger signal to coder B \n
///
///          Pin map:
/// \code
///           PIN | SCHEMATIC NAME | DIRECTION | ACTIVE
///           ----+----------------+-----------+-------
///           PA0 | (GREEN)        | OUTPUT    | LOW
///           PA1 | (RED)          | OUTPUT    | LOW
///           PA2 | SHUTDOWN       | OUTPUT    | R.EDGE
///           PA3 | A_SEL_3        | OUTPUT    | LOW
///           PA4 | A_SEL_4        | OUTPUT    | LOW
///           PA5 | A_SEL_1        | OUTPUT    | LOW
///           PA6 | A_SEL_2        | OUTPUT    | LOW
///           PA7 | A_SEL_5        | OUTPUT    | LOW
///           PB6 | SAFETY         | INPUT     | HIGH
///           PB7 | (BUZZER)       | OUTPUT    | HIGH
///           PD4 | CTO            | INPUT     | HIGH
///           PD5 | TEST_OK        | OUTPUT    | LOW
///           PD6 | RTX_OFF        | OUTPUT    | LOW
///           PD7 | RTX_SENSE      | INPUT     | LOW
///           PE0 | BUS1           | INPUT     | -
///           PE1 | BUS2           | INPUT     | -
///           PE2 | BUS3           | INPUT     | LOW
///           PE3 | BUS4           | INPUT     | HIGH
///           PE6 | A_STOP_PULSE   | OUTPUT    | LOW
///           PE7 | A_STOP_INPUT   | INPUT     | LOW
///           PG0 | RED_LED        | OUTPUT    | HIGH
///           PG1 | (JTAG_EN)      | INPUT     | LOW
/// \endcode
///
/// \par MAGIC NUMBERS:
/// Explicit numbers are used instead of #define'd macros throughout the code
/// wherever it's been considered easier to understand.
///
/// \par INTERFACE FUNCTIONS:
/// The driver has a high number of interface functions, mostly to set/reset
/// output pins and to get the status of input pins. This avoids the use of
/// global variables.
///
//  CHANGES Modified stop pulse management.
//
//============================================================================

#include  <ioavr.h>

#include "includes.h"
#include "Adc.h"
#include "Digital.h"

/*--------------------------------- Definitions ------------------------------*/

#undef  VAR_STATIC
#define VAR_STATIC static

#define MSK_DI                  (INT8U)0x0F         ///< Mask for digital input (D17-D20)
#define MASK_STOP               (INT8U)0x02         ///< Mask for STOP input

/*----------------------------------- Macros ----------------------------------*/

#define ENABLE_LATCH()          (PORTG |= 0x04)     ///< Set CMOS multiplexer latch signal
#define DISABLE_LATCH()         (PORTG &= ~0x04)    ///< Clear CMOS multiplexer latch signal
#define ENABLE_A_H_SEL()        (PORTA &= ~0x08)    ///< Select H directions multiplexer input
#define DISABLE_A_H_SEL()       (PORTA |= 0x08)     ///< Deselect H directions multiplexer input
#define ENABLE_A_L_SEL()        (PORTA &= ~0x10)    ///< Select L direction multiplexer input
#define DISABLE_A_L_SEL()       (PORTA |= 0x10)     ///< Deselect L direction multiplexer input
#define ENABLE_A_D2_SEL()       (PORTA &= ~0x20)    ///< Select D9-D16 multiplexer input
#define DISABLE_A_D2_SEL()      (PORTA |= 0x20)     ///< Deselect D9-D16 multiplexer input
#define ENABLE_A_D1_SEL()       (PORTA &= ~0x40)    ///< Select D1-D8 multiplexer input
#define DISABLE_A_D1_SEL()      (PORTA |= 0x40)     ///< Deselect D1-D8 multiplexer input
#define ENABLE_A_D3_SEL()       (PORTA &= ~0x80)    ///< Select D17-D20 multiplexer input
#define DISABLE_A_D3_SEL()      (PORTA |= 0x80)     ///< Deselect D17-D20 multiplexer input
#define SET_A_STOP_PULSE()      (PORTE |= 0x40)     ///< Set STOP_A output high
#define RESET_A_STOP_PULSE()    (PORTE &= ~0x40)    ///< Set STOP_A output low
#define SET_SHUTDOWN()          (PORTA |= 0x04)     ///< Shutdown output high
#define RESET_SHUTDOWN()        (PORTA &= ~0x04)    ///< Shutdown output low
#define BUZZER_ON()             (PORTB |= 0x80)     ///< Activate buzzer
#define BUZZER_OFF()            (PORTB &= ~0x80)    ///< Deactivate buzzer
#define LED_EXT_ON()            (PORTG |= 0x01)     ///< Turn on external red LED
#define LED_EXT_OFF()           (PORTG &= ~0x01)    ///< Turn off external red LED
#define LED_RED_ON()            (PORTA &= ~0x02)    ///< Turn on internal red LED
#define LED_RED_OFF()           (PORTA |= 0x02)     ///< Turn off internal red LED
#define RTX_ON()                (PORTD &= ~0x40)    ///< Turn on RTX power
#define RTX_OFF()               (PORTD |= 0x40)     ///< Turn off RTX power
#define TEST_OK_ON()            (PORTD &= ~0x20)    ///< Set TEST_OK output low
#define GET_DIGITAL_CMD()       ((INT8U)PINC)       ///< Read digital input port
#define GET_STOP_INPUT()        ((INT8U)(((PINE & 0x80) != 0) ? 0x02 : 0x00))   ///< Read STOP pushbutton
#define GET_SAFETY()            ((INT8U)(((PINB & 0x40) != 0) ? 0x01 : 0x00))   ///< Read SAFETY input
#define GET_RTX_SENSE()         ((INT8U)(((PIND & 0x80) != 0) ? 1 : 0))         ///< Check RTX power

/*------------------------------------ Const ---------------------------------*/

/*------------------------------------ Enums ---------------------------------*/

//! States for digital input reading state machine
enum {
    DIGITAL_TEST_RTX,   ///< Test RTX shutdown function
    DIGITAL_IDLE,       ///< Wait until new inputs reading is triggered
    DIGITAL_GET_H,      ///< Read High directions
    DIGITAL_GET_L,      ///< Read Low directions
    DIGITAL_GET_D1,     ///< Read digital commands D1-D8
    DIGITAL_GET_D2,     ///< Read digital commands D9-D16
    DIGITAL_GET_D3,     ///< Read digital commands D17-D20
    DIGITAL_FAIL        ///< Error state
};

//! Shutdown sequence states
enum { SHUTDOWN_SET, SHUTDOWN_RESET };

/*----------------------------------- Globals --------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC INT8U Digital_State;                 ///< State machine variable
VAR_STATIC INT8U Digital_Cmd[DIGITAL_CMD_NUM];  ///< Buffer for digital inputs
VAR_STATIC BOOLEAN8 Flg_Digital_Reading;        ///< Digital inputs reading in progress
VAR_STATIC BOOLEAN8 Flg_Digital_Failure;        ///< Digital fail
VAR_STATIC BOOLEAN8 Flg_Stop_Pulse;             ///< Time to generate pulse on STOP out

///< STOP button status
__no_init VAR_STATIC INT8U Digital_Stop     @ "SAFETY_RAM";     ///< Status of STOP input
__no_init VAR_STATIC INT8U Digital_Stop_Inv @ "SAFETY_RAM_REV"; ///< Status of STOP input, inverted

/*---------------------------------- Prototypes -------------------------------*/

/*-------------------------------- Inline Function ----------------------------*/

//! Short Delay.
#pragma inline=forced
static void Delay(void)
{
  INT16U i;

  for (i = 0; i < 10; i++) {
    ;
  }
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Initialization of digital I/O module
/// \remarks    -
//
//-----------------------------------------------------------------------------
void Digital_Init(void)
{
 INT8U i;

  DDRA |= 0xFC;                             // PA2-PA7 as outputs (SHUTDOWN,A_H_SEL,A_L_SEL,A_D2_SEL,A_D1_SEL,A_D3_SEL)
  PORTA &= ~0x04;                           // PA2 low (SHUTDOWN)
  PORTA |= 0xF8;                            // PA3-PA7 high (disable A_H_SEL,A_L_SEL,A_D2_SEL,A_D1_SEL,A_D3_SEL)
  DDRB &= ~(0x40);                          // PB6 as input (SAFETY command)
  DDRB |= 0x80;                             // PB7 as output (BUZZER)
  BUZZER_OFF();                             // PB7 low (turn off buzzer)
  PORTC = 0x00;                             // Disable pull-ups on port C
  DDRC = 0x00;                              // Port C as input (from CMOS latches)
  PORTD |= 0x20;                            // PD5 high (TEST_OK not active)
  DDRD |= 0x20;                             // PD5 as output (TEST_OK)
  DDRD |= 0x40;                             // PD6 as output (RTX_OFF)
  DDRD &= ~0x10;                            // PD4 as input (CTO)
  PORTE &= ~0x08;                           // PE3 low (disable trigger)
  DDRE |= 0x08;                             // PE3 as (trigger)
  DDRE &= ~(0x80);                          // PE7 as input (STOP command)
  DDRE |= 0x40;                             // PE6 as output (STOP pulse)
  DDRE &= ~0x04;                            // PE2 as input (coder B ready)
  DDRG |= 0x04;                             // PG2 as output (LATCH)
  DDRG |= 0x01;                             // PG0 as output (red LED)
  LED_EXT_OFF();                            // PG0 low (turn OFF red LED)
  DISABLE_LATCH();                          // PG2 low (disable LATCH)
  Flg_Digital_Reading = FALSE;              // Reading not in progress
  Flg_Stop_Pulse = FALSE;                   // STOP pulse timeout not elapsed
  Digital_Stop = 0;                         // STOP button pressed
  Digital_Stop_Inv = 0xFF;                  //
  Digital_State = DIGITAL_TEST_RTX;         // Init digital state machine
  for (i = 0; i < DIGITAL_CMD_NUM; i++) {   // Clear digital commands buffer
     Digital_Cmd[i] = (INT8U)0;
  }
  Flg_Digital_Failure = FALSE;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Forces Stateflow to enter ON state
/// \return     TRUE
/// \remarks    Needed by Stateflow
//
//-----------------------------------------------------------------------------
BOOLEAN8 Power(void)
{
  return TRUE;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Manages reading of digital inputs.
/// \remarks    -
//
//-----------------------------------------------------------------------------
void Digital_Manager(void)
{
  Ctrl_Flow_Cnt += CALLED_DIGITAL;

  if (Flg_Stop_Pulse == TRUE) {
    RESET_A_STOP_PULSE();                       // Generate STOP pulse
    Delay();
    Delay();
    SET_A_STOP_PULSE();
    Flg_Stop_Pulse = FALSE;
  } else {
    SET_A_STOP_PULSE();
  }
  switch (Digital_State) {
    case DIGITAL_TEST_RTX:
      if (GET_RTX_SENSE() == 1) {               // RTX power is on
        RTX_OFF();                              // Switch off RTX
        Delay();                                // Wait
        if (GET_RTX_SENSE() == 0) {             // RTX power is off
          RTX_ON();                             // Switch on RTX
        } else {                                // RTX can not be powerd off
          Flg_Digital_Failure = TRUE;
        }
      } else {                                  // RTX power is off
        Flg_Digital_Failure = TRUE;
      }
      Digital_State = DIGITAL_IDLE;             // RTX test passed
      break;

    case DIGITAL_GET_H:
      Digital_Cmd[DIGITAL_SAFETY] |= GET_STOP_INPUT(); // Read Stop
      Digital_Cmd[DIGITAL_SAFETY] |= GET_SAFETY();     // Read Safety
      Digital_Cmd[DIGITAL_DH] = GET_DIGITAL_CMD();      // Read A_H
      DISABLE_A_H_SEL();
      ENABLE_A_L_SEL();
      Digital_State = DIGITAL_GET_L;
      break;

    case DIGITAL_GET_L:
      Digital_Cmd[DIGITAL_SAFETY] |= GET_STOP_INPUT(); // Read Stop again
      Digital_Cmd[DIGITAL_SAFETY] |= GET_SAFETY();     // Read Safety again
      Digital_Cmd[DIGITAL_DL] = GET_DIGITAL_CMD();      // Read A_L
      DISABLE_A_L_SEL();
      ENABLE_A_D1_SEL();
      Digital_State = DIGITAL_GET_D1;
      break;

    case DIGITAL_GET_D1:
      Digital_Cmd[DIGITAL_SAFETY] |= GET_STOP_INPUT(); // Read Stop again
      Digital_Cmd[DIGITAL_SAFETY] |= GET_SAFETY();     // Read Safety again
      Digital_Cmd[DIGITAL_D1] = GET_DIGITAL_CMD();     // Read A_D1
      DISABLE_A_D1_SEL();
      ENABLE_A_D2_SEL();
      Digital_State = DIGITAL_GET_D2;
      break;

    case DIGITAL_GET_D2:
      Digital_Cmd[DIGITAL_SAFETY] |= GET_STOP_INPUT(); // Read Stop again
      Digital_Cmd[DIGITAL_SAFETY] |= GET_SAFETY();     // Read Safety again
                                                       // Get STOP command status
      Digital_Stop = (INT8U)(Digital_Cmd[DIGITAL_SAFETY] & MASK_STOP) >> 1;
      Digital_Stop_Inv = ~Digital_Stop;
      Digital_Cmd[DIGITAL_D2] = GET_DIGITAL_CMD();     // Read A_D2
      DISABLE_A_D2_SEL();
      ENABLE_A_D3_SEL();
      Flg_Digital_Reading = FALSE;
      Digital_State = DIGITAL_GET_D3;
      break;

    case DIGITAL_GET_D3:
      Digital_Cmd[DIGITAL_D3] = (GET_DIGITAL_CMD() & MSK_DI); // Read A_D3
      DISABLE_A_D3_SEL();
      Digital_State = DIGITAL_IDLE;
      break;

    case DIGITAL_FAIL:
      break;

    case DIGITAL_IDLE:                                  // Wait until next sampling trigger
    default:
      if (Is_Sampling() == TRUE) {                      // Start new digital command reading
        ENABLE_LATCH();                                 // Generate a Dir latch pulse
        Digital_State = DIGITAL_GET_H;
        Digital_Cmd[DIGITAL_SAFETY] = GET_STOP_INPUT(); // Read Stop
        Digital_Cmd[DIGITAL_SAFETY] |= GET_SAFETY();    // Read Safety
        DISABLE_LATCH();
        Flg_Digital_Reading = TRUE;
        ENABLE_A_H_SEL();
      }
      break;
  }

  Ctrl_Flow_Cnt_Inv -= CALLED_DIGITAL;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Digital state machine is in error state
/// \return     TRUE if Flg_Digital_Failure is set, FALSE otherwise
/// \remarks    -
//
//-----------------------------------------------------------------------------
BOOLEAN8 Is_Digital_Fail(void)
{
  return Flg_Digital_Failure;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Digital state machine is reading inputs
/// \return     TRUE if digital state machine is busy reading inputs,
///             FALSE otherwise
/// \remarks    -
//
//-----------------------------------------------------------------------------
BOOLEAN8 Is_Dig_Reading(void)
{
  return Flg_Digital_Reading;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Get one byte of digital inputs
/// \param      index index of digital input buffer
/// \return     digital input byte, specified by index
/// \remarks    -
//
//-----------------------------------------------------------------------------
INT8U Digital_Get_Cmd(INT8U index)
{
  INT8U ret = 0;

  if (index < DIGITAL_CMD_NUM) {   // Array limit check
    ret = Digital_Cmd[index];
  }
  return ret;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Status of STOP button
/// \return     FALSE if STOP is pressed or check of safety variable failed,
///             TRUE if STOP is released
/// \remarks    Pulses on STOP switches are filtered
//
//-----------------------------------------------------------------------------
BOOLEAN8 In_Stop(void)
{
  BOOLEAN8 status;

  if ((Digital_Stop ^ Digital_Stop_Inv) == 0xFF) {
    status = (BOOLEAN8)Digital_Stop;
  } else {
    status = FALSE;
  }
  return status;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Enable generation of pulse on STOP output
/// \remarks    -
//
//-----------------------------------------------------------------------------
void Stop_Pulse_On(void)
{
  Flg_Stop_Pulse = TRUE;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Disable generation of pulse on STOP output
/// \remarks    -
//
//-----------------------------------------------------------------------------
void Stop_Pulse_Off(void)
{
  Flg_Stop_Pulse = FALSE;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Set / reset trigger signal to coder B
/// \param      mode = TRUE set trigger signal, FALSE reset trigger signal
/// \remarks    -
//
//-----------------------------------------------------------------------------
void Coder_A_Ready(BOOLEAN8 mode)
{
 if (mode == TRUE) {
   PORTE |= 0x08;                  // Set trigger
 } else {
   PORTE &= ~0x08;                 // Reset trigger
 }
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Read coder B ready signal
/// \return     TRUE if coder B ready is high, FALSE otherwise
/// \remarks    -
//
//-----------------------------------------------------------------------------
BOOLEAN8 In_Coder_B_Ready(void)
{
  return ((BOOLEAN8)(((PINE & 0x04) != 0) ? TRUE : FALSE));
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Read CTO signal (Consent To Operate)
/// \return     TRUE if CTO signal is high, FALSE otherwise
/// \remarks    -
//
//-----------------------------------------------------------------------------
BOOLEAN8 In_CTO(void)
{
  return ((BOOLEAN8)(((PIND & 0x10) != 0) ? TRUE : FALSE));
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Activate TEST_OK output
/// \remarks    -
//
//-----------------------------------------------------------------------------
void Test_OK_On(void)
{
   TEST_OK_ON();
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Turn on external red LED and buzzer
/// \remarks    -
//
//-----------------------------------------------------------------------------
void Led_On(void)
{
#ifndef DEBUG_BUZZER   // In debug mode silent buzzer
  BUZZER_ON();
#endif
  LED_EXT_ON();
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Turn off external red LED and buzzer
/// \remarks    -
//
//-----------------------------------------------------------------------------
void Led_Off(void)
{
  BUZZER_OFF();
  LED_EXT_OFF();
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Shut down RTX power
/// \return     -
/// \remarks    -
//
//-----------------------------------------------------------------------------
void Shut_Down_RTX(void)
{
  RTX_OFF();
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Turn on internal red LED
/// \remarks    -
//
//-----------------------------------------------------------------------------
void Led_Red_On(void)
{
  LED_RED_ON();
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Turn off internal red LED
/// \remarks    -
//
//-----------------------------------------------------------------------------
void Led_Red_Off(void)
{
  LED_RED_OFF();
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Shut down the power of the transmitting unit
/// \return     -
/// \remarks    Power is switched off on rising edge of shut down pulse.
//
//-----------------------------------------------------------------------------
void Shut_Down_TU(void)
{
#ifndef DEBUG_SHUTDN     // Normal operation mode
  static INT8U shutdown_status = SHUTDOWN_RESET;

  switch (shutdown_status) {
    case SHUTDOWN_RESET:
      RESET_SHUTDOWN();
      shutdown_status = SHUTDOWN_SET;
      break;

    case SHUTDOWN_SET:
      SET_SHUTDOWN();
      shutdown_status = SHUTDOWN_RESET;
      break;

    default:
      shutdown_status = SHUTDOWN_RESET;
      break;
  }
#endif
}
