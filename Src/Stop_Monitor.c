//============================================================================+
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/Stop_Monitor.c $
// $Revision: 9742 $
// $Date: 2011-02-09 10:05:27 +0100 (mer, 09 feb 2011) $
// $Author: micheleb $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief STOP monitor module.
///
/// \file
///       The status of STOP button is read on the A_STOP_INPUT pin:
///
///         - when A_STOP_INPUT is at logic 0, STOP button is pressed \n
///         - when A_STOP_INPUT is at logic 1, STOP button is released
///
///       A_STOP_INPUT pin is connected through STOP switch to the pin
///       B_STOP_PULSE of coder B.
///
///       Coder B sends short pulses on B_STOP_PULSE with a period between
///       113 ms and 123 ms.
///
///       Pulses reach A_STOP_INPUT pin only when STOP button is released.
///       The time between pulses ("pulse width") is measured by means of
///       a capture timer configured in continuous counting mode.
///
///       The pulse width is computed as the difference between two
///       consecutive values of the capture register.
///
///       The result is accurate even if one (and only one) timer overflow
///       occurs between two consecutive capture events.
///
///       If two or more overflows occur, the pulse are missing regardless
///       of the difference between capture values.
///
///       Timer overflows are counted inside a interrupt routine. To avoid
///       false errors, the counter is increased only if the STOP button
///       is released.
///
///       Following table summarizes the different scenarios considered
///       in the code:
/// \code
///          | PULSE WIDTH    |              SCENARIO
///       ---+----------------+-----------------------------------------------
///        1 |     = 0 ms     | Pulse width multiple of overflow period, see 5
///       ---+----------------+-----------------------------------------------
///        2 |    < 10 ms     | Switch bounce when STOP button is released
///       ---+----------------+-----------------------------------------------
///        3 | [10 - 118[ ms  | Possible short between STOP switches
///       ---+----------------+-----------------------------------------------
///        4 | [118 - 123] ms | OK
///       ---+----------------+-----------------------------------------------
///        5 |    > 123 ms    | Possible stuck @ 1 on A_STOP_INPUT
///       ---+----------------+-----------------------------------------------
/// \endcode
///
/// \par FUNCTION CALL IN ISR:
/// Interrupt routine Timer3_Ovf_ISR() calls function In_Stop() to know the
/// status of STOP pushbutton.\n\n
/// Using a function call avoids sharing global variables between modules.\n\n
/// In_Stop() has no side effects, it simply returns the value of a variable
/// local to Digital module.\n\n
/// Variable is of type BOOLEAN8, thus operations on it can be treated as atomic.
/// \n\n
//
// CHANGES doxygen
//
//============================================================================*/

#include  <ioavr.h>

#include "includes.h"
#include "Digital.h"
#include "Stop_Monitor.h"

/*--------------------------------- Definitions ------------------------------*/

#undef  VAR_STATIC
#define VAR_STATIC static

#define TMR3_FDIV8        (INT8U)2       ///< Timer 3 clock divisor
#define MAX_MISSED_PULSES (INT8U)3       ///< Maximum number of missing pulses
#define MAX_WRONG_PULSES  (INT8U)5       ///< Maximum number of wrong pulses

#define MIN_BOUND         (INT16U)5000   ///< Absolute lower limit of pulse width
#define LOWER_BOUND       (INT16U)56500  ///< Lower limit for correct pulse width
#define UPPER_BOUND       (INT16U)61500  ///< Upper limit for correct pulse width

/*------------------------------------ Macros --------------------------------*/

#define CLEAR_TIMER3_INTS()     (ETIFR |= 0x3E) ///< Clear all TIMER3 interrupt flags
                                /* XX11111X  Timer interrupt flag register
                                   |||||     MUST WRITE "1" TO CLEAR BITS
                                   ||||+-- OCF3C  Tmr 3 cmp C int. flag
                                   |||+--- TOV3   Tmr 3 ovfw int. flag
                                   ||+---- OCF3B  Tmr 3 cmp B int. flag
                                   |+----- OCF3A  Tmr 3 cmp A int. flag
                                   +------ ICF3   Tmr 3 capt. int. flag   */
#define CLEAR_CAPTURE_FLAG()    (ETIFR |= (1<<ICF3))  ///< Clear TIMER3 capture interrupt flag
                                /* XX1XXXXX  Timer interrupt flag register
                                     |||||   MUST WRITE "1" TO CLEAR BITS
                                     ||||+-- OCF3C  Tmr 3 cmp C int. flag
                                     |||+--- TOV3   Tmr 3 ovfw int. flag
                                     ||+---- OCF3B  Tmr 3 cmp B int. flag
                                     |+----- OCF3A  Tmr 3 cmp A int. flag
                                     +------ ICF3   Tmr 3 capt. int. flag   */
#define IS_CAPTURE_FLAG()       (((ETIFR & (1<<ICF3)) != 0) ? 1 : 0) ///< TIMER3 capture interrupt flag is active
                                /* XX1XXXXX  Timer interrupt flag register
                                   |||||     MUST WRITE "1" TO CLEAR BITS
                                   ||||+-- OCF3C  Tmr 3 cmp C int. flag
                                   |||+--- TOV3   Tmr 3 ovfw int. flag
                                   ||+---- OCF3B  Tmr 3 cmp B int. flag
                                   |+----- OCF3A  Tmr 3 cmp A int. flag
                                   +------ ICF3   Tmr 3 capt. int. flag   */
#define DISABLE_TIMER3_INTS()   (ETIMSK &= 0xC3)  ///< Disable all TIMER3 interrupts
                                /* XX00000X   Timer interrupt mask register
                                   |||||
                                   ||||+-- OCF3C  Tmr 3 cmp C int. flag
                                   |||+--- TOIE3  Tmr 3 ovfw int. disable
                                   ||+---- OCIE3B Tmr 3 cmp B int. disable
                                   |+----- OCIE3A Tmr 3 cmp A int. disable
                                   +------ TICIE3 Tmr 3 capt. int. disable */
#define ENABLE_OVF3_INT()       (ETIMSK |= (1<<TOV3)) ///< Enable TIMER3 capture interrupt
#define INIT_TCCR3A()           (TCCR3A = (INT8U)0)   ///< Initialize TIMER3 Control Register A
                                /* 00000000 Timer/Counter control register A
                                   ||||||||
                                   |||||||+- WGM30  Tmr/cntr normal mode
                                   ||||||+-- WGM31  Tmr/cntr normal mode
                                   |||||+--- COM3C0 OC3C disconnected
                                   ||||+---- COM3C1 OC3C disconnected
                                   |||+----- COM3B0 OC3B disconnected
                                   ||+------ COM3B1 OC3B disconnected
                                   |+------- COM3A0 OC3A disconnected
                                   +-------- COM3A1 OC3A disconnected        */
#define INIT_TCCR3B()           (TCCR3B = 0x80) ///< Init TIMER3 Control Register B
                                /* 10000000 Timer/Counter control register B
                                   ||||||||
                                   |||||||+- CS30   \
                                   ||||||+-- CS31   | Timer/counter stopped
                                   |||||+--- CS32   /
                                   ||||+---- WGM32  Timer/counter normal mode
                                   |||+----- WGM33  Timer/counter normal mode
                                   ||+------ -
                                   |+------- ICES3  Input capture edge select
                                   +-------- ICNC3  Noise canceller          */
#define START_TIMER3(f)         (TCCR3B |= (f)) ///< Start TIMER3, with prescaler f
                                /* 1XXXXfff Timer/Counter control register B
                                   ||||||||
                                   |||||||+- CS30   \
                                   ||||||+-- CS31   | Timer clock
                                   |||||+--- CS32   /
                                   ||||+---- WGM32  Timer/counter normal mode
                                   |||+----- WGM33  Timer/counter normal mode
                                   ||+------ -
                                   |+------- ICES3  Input capture edge
                                   +-------- ICNC3  Noise canceller          */
#define SET_FALLING_EDGE_CAPT() (TCCR3B &= ~(1<<ICES3)) ///< Select falling capture edge

/*--------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals --------------------------------*/

/*----------------------------------- Locals --------------------------------*/

VAR_STATIC INT16U A_Stop_Pulse_Ovf;            ///< Timer 3 overflows counter
VAR_STATIC BOOLEAN8 Flg_A_Stop_Pulse_Missing;  ///< STOP input is high and no pulses are detected
VAR_STATIC BOOLEAN8 Flg_A_Stop_Pulse_Wrong;    ///< STOP input is high and pulse frequency is wrong

/*--------------------------------- Prototypes -------------------------------*/

//----------------------------------------------------------------------------
//
//  DESCRIPTION STOP monitor initialization
/// \remarks    Timer 3 is configured for input capture on falling edges of
///             A_STOP_INPUT signal.
//
//----------------------------------------------------------------------------
void Stop_Monitor_Init(void)
{
  INIT_TCCR3A();                    // Initialize timer 3 control register A
  INIT_TCCR3B();                    // Initialize timer 3 control register B
  DISABLE_TIMER3_INTS();            // Disable all interrupts of timer 3
  CLEAR_TIMER3_INTS();              // Clear all interrupt flags of timer 3
  TCNT3H = 0;                       // Clear timer 3 counter (high byte first)
  TCNT3L = 0;
  ICR3 = 0x0000;                    // Clear timer 3 capture register
  SET_FALLING_EDGE_CAPT();          // Select capture on falling edge
  DDRE &= ~0x80;                    // PE7 (ICP3) as input
  A_Stop_Pulse_Ovf = 0;             // Clear counter of timer 3 overflows
  Flg_A_Stop_Pulse_Missing = FALSE; // Clear missing pulses flag
  Flg_A_Stop_Pulse_Wrong = FALSE;   // Clear wrong pulses flag
  ENABLE_OVF3_INT();                // Enable timer 3 overflow interrupt
  START_TIMER3(TMR3_FDIV8);         // Start timer 3 with prescaler
}

//----------------------------------------------------------------------------
//
//  DESCRIPTION Interrupt service routine for timer 3 overflow
/// \remarks    Counts timer 3 overflows only if STOP button is released.
///             In_Stop() function must be reentrant or called only inside
///             this interrupt routine.
//
//----------------------------------------------------------------------------
//lint --e{528} suppress "Timer3_Ovf_ISR() not referenced"
#pragma vector=TIMER3_OVF_vect
static __interrupt void Timer3_Ovf_ISR(void)
{
    if (In_Stop() != 0) {
       if (A_Stop_Pulse_Ovf < MAX_MISSED_PULSES) {
          A_Stop_Pulse_Ovf++;
       }
    }
}

//----------------------------------------------------------------------------
//
//  DESCRIPTION Monitors STOP status
/// \remarks    -
//
//----------------------------------------------------------------------------
void Stop_Monitor_Manager(void)
{
  VAR_STATIC INT16U old_ccr = 0;                // Previous value of capture register
  VAR_STATIC INT8U wrong_pulses = 0;            // Counter for wrong pulses
  INT16U new_ccr;                               // Current value of capture register
  INT16U pulse_width;                           // Width of STOP pulses

  Ctrl_Flow_Cnt += CALLED_STOP;

  if (In_Stop() == 0) {                         // STOP is pressed
    wrong_pulses = 0;                           // Clear counter of wrong pulses
  }
  if (IS_CAPTURE_FLAG() != 0) {                 // Pulse has been detected
    new_ccr = (INT16U) ICR3;                    // Save captured timer value
    pulse_width = (new_ccr - old_ccr);          // Compute STOP pulse width
    old_ccr = new_ccr;                          // Update previously captured timer value
    CLEAR_CAPTURE_FLAG();                       // Clear capture event flag
    if (pulse_width > 0) {                      // STOP pulse width is non zero
      if ((A_Stop_Pulse_Ovf > 1) ||             // At least two overflows occurred
         ((pulse_width > MIN_BOUND) &&          // Pulse is not due to a switch bounce
          (pulse_width < LOWER_BOUND)) ||       // Pulse width is too short
          (pulse_width > UPPER_BOUND)) {        // Pulse width is too long
        if (wrong_pulses < MAX_WRONG_PULSES) {  // Clip wrong pulses counter
           wrong_pulses++;                      // Increase wrong pulses counter
        }
      } else {                                  // Right pulse width
        wrong_pulses = 0;                       // Clear wrong pulse counter
      }
    } else {                                    // Pulse width is 0 or multiple of timer overflow period
        if (wrong_pulses < MAX_WRONG_PULSES) {  // Clip wrong pulses counter
           wrong_pulses++;                      // Increase wrong pulses counter
        }
    }
    A_Stop_Pulse_Ovf = 0;                       // Reset overflow counter
  }

  if (wrong_pulses >= MAX_WRONG_PULSES) {       // Too many wrong pulses
    Flg_A_Stop_Pulse_Wrong = TRUE;              // Set wrong pulse flag
  } else {
    Flg_A_Stop_Pulse_Wrong = FALSE;             // Clear wrong pulse flag
  }

  if (A_Stop_Pulse_Ovf >= MAX_MISSED_PULSES) {  // Too many missed pulses
    Flg_A_Stop_Pulse_Missing = TRUE;            // Set missing pulse flag
  } else {
    Flg_A_Stop_Pulse_Missing = FALSE;           // Clear missing pulse flag
  }

  Ctrl_Flow_Cnt_Inv -= CALLED_STOP;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Stop monitor is in error condition.
/// \return     Error code (no error, missing pulse or wrong pulse)
/// \remarks    -
//
//-----------------------------------------------------------------------------
INT8U Is_Stop_Mon_Fail(void)
{
  INT8U err_code;

  if (Flg_A_Stop_Pulse_Missing == TRUE) {
    err_code = STOP_FAIL_PULSE_MISSING;
  } else if (Flg_A_Stop_Pulse_Wrong == TRUE) {
    err_code = STOP_FAIL_PULSE_WRONG;
  } else {
    err_code = STOP_NO_FAIL;
  }
  return err_code;
}
