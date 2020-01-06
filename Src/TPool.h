//============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/TPool.h $
// $Revision: 9466 $
// $Date: 2011-01-13 15:01:29 +0100 (gio, 13 gen 2011) $
// $Author: micheleb $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief Software timers.
///
/// \file
///
// CHANGES Added enum for timestamp timer.
//
//
//============================================================================

/*-------------------------------- Definitions -------------------------------*/

#ifndef TPOOL_H
#define TPOOL_H

#undef VAR_GLOBAL
#define VAR_GLOBAL extern

#define TS_CLK_FREQUENCY_HZ ((INT32U) 4000000)  //!< External crystal frequency.
#define TS_TICK_RATE_HZ     ((INT32U)244)       //!< Timer tick rate (4.098 ms)
#define TMR_MAXDELAY        ((INT16U)0xFFFE)    //!< Max value for timer counters

// Delay values greater than 4.47 min are not allowed, because of timer overflow
#define TMR_4_m     (INT16U)((TS_TICK_RATE_HZ * 240) + 0.5)    ///< Value for 4 minutes
#define TMR_3_5_m   (INT16U)((TS_TICK_RATE_HZ * 210) + 0.5)    ///< Value for 3.5 minutes
#define TMR_1_m     (INT16U)((TS_TICK_RATE_HZ * 60) + 0.5)     ///< Value for 4 minutes
#define TMR_30_s    (INT16U)((TS_TICK_RATE_HZ * 30) + 0.5)     ///< Value for 30 seconds
#define TMR_15_s    (INT16U)((TS_TICK_RATE_HZ * 15) + 0.5)     ///< Value for 15 seconds
#define TMR_5_s     (INT16U)((TS_TICK_RATE_HZ * 5) + 0.5)      ///< Value for 5 seconds
#define TMR_3_5_s   (INT16U)((TS_TICK_RATE_HZ * 3.5) + 0.5)    ///< Value for 3.5 second
#define TMR_2_5_s   (INT16U)((TS_TICK_RATE_HZ * 2.5) + 0.5)    ///< Value for 2.5 second
#define TMR_2_s     (INT16U)((TS_TICK_RATE_HZ * 2) + 0.5)      ///< Value for 2 second
#define TMR_1_5_s   (INT16U)((TS_TICK_RATE_HZ * 1.5) + 0.5)    ///< Value for 1.5 second
#define TMR_1_s     (INT16U)((TS_TICK_RATE_HZ * 1) + 0.5)      ///< Value for 1 second
#define TMR_800_ms  (INT16U)((TS_TICK_RATE_HZ * 0.800) + 0.5)  ///< Value for 800 milliseconds
#define TMR_500_ms  (INT16U)((TS_TICK_RATE_HZ * 0.500) + 0.5)  ///< Value for 500 milliseconds
#define TMR_250_ms  (INT16U)((TS_TICK_RATE_HZ * 0.250) + 0.5)  ///< Value for 250 milliseconds
#define TMR_200_ms  (INT16U)((TS_TICK_RATE_HZ * 0.200) + 0.5)  ///< Value for 200 milliseconds
#define TMR_100_ms  (INT16U)((TS_TICK_RATE_HZ * 0.100) + 0.5)  ///< Value for 100 milliseconds
#define TMR_94_ms   (INT16U)((TS_TICK_RATE_HZ * 0.094) + 0.5)  ///< Value for 94 milliseconds
#define TMR_50_ms   (INT16U)((TS_TICK_RATE_HZ * 0.050) + 0.5)  ///< Value for 50 milliseconds
#define TMR_15_ms   (INT16U)((TS_TICK_RATE_HZ * 0.015) + 0.5)  ///< Value for 15 milliseconds
#define TMR_10_ms   (INT16U)((TS_TICK_RATE_HZ * 0.011) + 0.5)  ///< Value for 10 milliseconds

/*-------------------------------- Constants --------------------------------*/

/*---------------------------------- Enums ----------------------------------*/

//! General purpose timers
enum {
  TMR_BATTERY,       ///< Sampling period of battery voltage
  TMR_TWI_TO,        ///< TWI communication time-out
  TMR_TWI_RECOVERY,  ///< TWI recovery timer from SDA line error
  TMR_STOP_PULSE,    ///< Stop A pulse period
  TMR_HIC,           ///< Hic communication Time-out
  TMR_TSTAMP,        ///< Timestamp timer
  TMR_NUM            ///< Number of timers
};

/*--------------------------------- Globals ---------------------------------*/

/*---------------------------------- Types ----------------------------------*/

/*-------------------------------- Interface --------------------------------*/

/// @defgroup TPOOL Timer pool driver
/// @{

void TPool_Init(void);              ///< Initialize general purpose timer pool
void TPool_Start(INT8U, INT16U);    ///< Starts selected timer
BOOLEAN8 TPool_End(INT8U);          ///< Check if selected timer has elapsed

/// @}

/*----------------------------------------------------------------------------*/

#endif

