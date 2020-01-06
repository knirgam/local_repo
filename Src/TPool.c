//============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/TPool.c $
// $Revision: 8855 $
// $Date: 2010-10-29 10:35:16 +0200 (ven, 29 ott 2010) $
// $Author: lorenzo $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief Software timers.
///
/// \file
/// \par Working principle:
/// A free running counter is increased inside a timer interrupt routine.\n\n
/// When n-th timer is started with delay D, D is added to the value of the
/// n-th timer register.\n\n
///   If the result is greater than the free running counter, it is copied into
/// the register of n-th timer: n-th register = free running + D.\n\n
///   If the result is lower than the free running counter, then an overflow in
///   the sum occured and all timer registers must be updated:
///  - if j-th register is greater than free running counter, j-th timer hasn't
///       expired and its value is updated with the number of remaining ticks:
///    j-th register = j-th register - free running.
///  - if j-th register is less than or equal to free running counter,
///    j-th timer has expired and j-th register is forced to 0.
///  - in both cases, the free running counter is reset to 0.
/// \par
///   When all timer registers have been updated and the free running counter
/// has been reset, the n-th register is updated with its expiring tick
/// period: n-th register = D.\n\n
/// N-th timer expires when free running counter reaches the value of
/// n-th register.
///
/// \par COMMON RESOURCES:
/// Variable "Ticks" of type INT16U is modified by TPool_Start(), TPool_End()
/// functions and by Tim_Sys_ISR() interrupt routine. \n\n
/// Operations on 16 bit variables are not atomic when performed on an 8 bit
/// microcontroller, and must be included in critical sections. \n\n
/// Therefore Timer_Sys interrupt is disabled during operations on "Ticks".
///
// CHANGES Renamed timer pool functions
//         Timer_Sys_Init() merged into TPool_Init()
//         Code inspection: names, header, common resources
//
//============================================================================

#include  <ioavr.h>

#include "includes.h"
#include "TPool.h"

/*-------------------------------- Definitions ------------------------------*/

#undef  VAR_STATIC
#define VAR_STATIC static

#define TMR2_FDIV64     3       ///< Timer 2 clock frequency divisor

/*---------------------------------- Macros ----------------------------------*/

#define CLEAR_TIMER2_INTS()     (TIFR |= 0xC0)  ///< Clear all interrupt flags of timer 2
                                /* 11XXXXXX    Timer interrupt flag register
                                   ||          MUST WRITE "1" TO CLEAR BITS
                                   |+--------- TOV2   Tmr 2 ovfw int. flag
                                   +---------- OCF2   Tmr 2 cmp int. flag */
#define DISABLE_TIMER2_INTS()   (TIMSK &= 0x3F) ///< Disable all interrupts of timer 2
                                /* 00XXXXXX   Timer interrupt mask register
                                   ||
                                   |+-------- TOIE2  Tmr 2 ovfw int. disable
                                   +--------- OCIE2  Tmr 2 cmp  int. disable */
#define INIT_TCCR2()            (TCCR2 = 0x48)  ///< Initialize timer 2 Control Register
                                /* 01001000 Timer/Counter 2 control register
                                   ||||||||
                                   |||||||+- CS20   \
                                   ||||||+-- CS21   | Timer/counter stopped
                                   |||||+--- CS22   /
                                   ||||+---- WGM21  Waveform generation mode
                                   |||+----- COM20  Compare match output mode
                                   ||+------ COM21  Compare match output mode
                                   |+------- WGM20  Waveform generation mode
                                   +-------- FOC2   Force output compare     */
#define START_TIMER2(p)         (TCCR2 |= (p))  ///< Start timer 2 with prescaler value 'p'
                                /* 0XXXXfff Timer/Counter 2 control register
                                   ||||||||
                                   |||||||+- CS20   \
                                   ||||||+-- CS21   | Timer clock select
                                   |||||+--- CS22   /
                                   ||||+---- WGM21  Waveform generation mode
                                   |||+----- COM20  Compare match output mode
                                   ||+------ COM21  Compare match output mode
                                   |+------- WGM20  Waveform generation mode
                                   +-------- FOC2   Force output compare     */
#define RESET_PRESCALER()       (SFIOR |= (1<<PSR321))  ///< Reset prescaler of timers 1, 2, 3
                                /* X---XXX1 Special Function IO Register
                                   |
                                   +-PSR321 */
#define CLEAR_TIMER2()          (TCNT2 = 0)  ///< Clear timer 2 counter
#define DISABLE_TIMER_SYS_INT() (TIMSK &= ~(1<<TOIE2))  ///< Disable Timer 2 overflow interrupt
#define ENABLE_TIMER_SYS_INT()  (TIMSK |= (1<<TOIE2))   ///< Enable Timer 2 overflow interrupt

/*----------------------------------- Types ---------------------------------*/

/*---------------------------------- Globals --------------------------------*/

/*----------------------------------- Locals --------------------------------*/

VAR_STATIC INT16U GP_Timer[TMR_NUM];    //!< General purpose timers pool.
VAR_STATIC volatile INT16U Ticks;       //!< Free running tick counter.

/*--------------------------------- Prototypes ------------------------------*/

//-----------------------------------------------------------------------------
//
// DESCRIPTION  Initialize tick timer and timer pool.
/// \remarks    -
//
//-----------------------------------------------------------------------------
void TPool_Init(void)
{
  INT8U j;

  DISABLE_TIMER2_INTS();        // Disable all interrupts of timer 2
  CLEAR_TIMER2_INTS();          // Clear all timer 2 interrupt flags
  INIT_TCCR2();                 // Initialize timer 2, counter is stopped
  CLEAR_TIMER2();               // Clear timer 2 counter
  ENABLE_TIMER_SYS_INT();       // Enable timer 2 overflow interrupt
  START_TIMER2(TMR2_FDIV64);    // Start timer 2 with prescaler 64
  RESET_PRESCALER();            // Reset prescaler of timers 1, 2 and 3

  for (j = 0; j < TMR_NUM; j++) {
    GP_Timer[j] = (INT16U) 0;   // Initialize timer array
  }
  Ticks = TMR_MAXDELAY;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION  Interrupt service routine for timer 2 overflow
/// \remarks     Updates system tick counter.
//
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Tim_Sys_ISR() not referenced"
#pragma vector=TIMER2_OVF_vect
static __interrupt void Tim_Sys_ISR(void)
{
  if (Ticks <= TMR_MAXDELAY) {
    Ticks++;                     // Increase system tick counter
  }
}

//-----------------------------------------------------------------------------
//
// DESCRIPTION  Starts selected timer with specified delay.
/// \param      timer_num timer to be started.
/// \param      delay timer period.
/// \remarks    Variable "Ticks" is a common resource: since 16 bit operations
///             are not atomic when performed on an 8 bit microcontroller,
///             it is necessary to disable Timer_Sys interrupt when modifying
///             variable "Ticks".
//
//-----------------------------------------------------------------------------
void TPool_Start(INT8U timer_num, INT16U delay)
{
  INT8U i;
  INT16U temp_delay;
  INT16U temp_ticks;

  if ((delay > 0) &&                        // Delay is not zero
      (timer_num < TMR_NUM)) {              // Valid timer number
                                            /* Begin of critical section */
    DISABLE_TIMER_SYS_INT();                // Disable interrupt while reading ticks
    temp_delay = delay + Ticks;             // Add delay to actual timer value
    ENABLE_TIMER_SYS_INT();                 // Re-enable Timer_Sys interrupt
                                            /* End of critical section */
    if (temp_delay < delay) {               // Overflow
                                            /* Begin of critical section */
      DISABLE_TIMER_SYS_INT();              // Disable interrupt while reading ticks
      temp_ticks = Ticks;                   // Save current tick number
      Ticks -= temp_ticks;                  // Decrease free running counter
      ENABLE_TIMER_SYS_INT();               // Enable Timer_Sys interrupt
                                            /* End of critical section */
      for (i = 0; i < TMR_NUM; i++) {       // Check all timers
        if (GP_Timer[i] > temp_ticks) {     // Timer not elapsed
          GP_Timer[i] -= temp_ticks;        // Subtract time to expiration
        } else {                            // Timer elapsed
          GP_Timer[i] = 0;                  // Reset timer
        }
      }
                                            /* Begin of critical section */
      DISABLE_TIMER_SYS_INT();              // Disable Timer_Sys interrupt
      temp_delay = delay + Ticks;           // Compute expiration time
      ENABLE_TIMER_SYS_INT();               // Re-enable Timer_Sys interrupt
                                            /* End of critical section */
    }
    GP_Timer[timer_num] = temp_delay;       // Update timer
  }
}

//-----------------------------------------------------------------------------
//
// DESCRIPTION Check if selected timer has elapsed.
/// \param     timer_num timer to be checked
/// \return    TRUE if timer has expired or timer_num is not a valid timer,
///            FALSE otherwise.
/// \remarks   -
//
//-----------------------------------------------------------------------------
BOOLEAN8 TPool_End(INT8U timer_num)
{
  BOOLEAN8 expired = TRUE;

  if (timer_num < TMR_NUM) {                  // Valid timer number
                                              /* Begin of critical section */
    DISABLE_TIMER_SYS_INT();                  // Disable Timer_Sys interrupt
    expired = (Ticks >= GP_Timer[timer_num]); // Check if timer elapsed
    ENABLE_TIMER_SYS_INT();                   // Re-enable Timer_Sys interrupt
                                              /* End of critical section */
  }
  return expired;
}

