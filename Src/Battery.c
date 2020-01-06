//============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/Battery.c $
// $Revision: 9742 $
// $Date: 2011-02-09 10:05:27 +0100 (mer, 09 feb 2011) $
// $Author: lorenzo $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief Battery management module
///
/// \file
///
/// \par COMMON RESOURCES:
/// - ADC driver uses TWI to read battery voltage from external ADC
/// - UID manager uses TWI to read code from external EEPROM
///
/// See TWI manager description for working priciple and contention management.
///
/// \par MAGIC NUMBERS:
/// Explicit numbers are used instead of #define'd macros throughout the code
/// wherever it's been considered easier to understand.
///
// CHANGES doxygen
//
//============================================================================

#include  <ioavr.h>

#include "includes.h"
#include "TPool.h"
#include "TWI_Master.h"
#include "Battery.h"

/*--------------------------------- Definitions ------------------------------*/

#undef  VAR_STATIC
#define VAR_STATIC static

#define NOMINAL_THR       (INT8U)0xBA    ///< Nominal battery voltage (~7.2 V)
#define BATTERY_ADC_ADDR  (INT8U)0x52    ///< Slave address of external I2C ADC
#define BATT_START_ADDR   (INT16U)0      ///< Starting address of external I2C ADC
#define BATT_ADC_NUM      (INT8U)2       ///< Number of bytes to read from external I2C ADC
#define NUM_SAMPLE        4              ///< Number of samples for computing average

#define MAX_SAMPLE        (INT8U)(NUM_SAMPLE-1) ///< Max index for samples buffer
#if ((NUM_SAMPLE ^ 1) && (NUM_SAMPLE ^ 2) && (NUM_SAMPLE ^ 4) && (NUM_SAMPLE ^ 8) && (NUM_SAMPLE ^ 16) && (NUM_SAMPLE ^ 32) && (NUM_SAMPLE ^ 64) && (NUM_SAMPLE ^ 128))
#   error Sample number is not a power of 2
#elif (NUM_SAMPLE == 1)
#   define AVG_SHIFT 0                  ///< Average division factor
#elif (NUM_SAMPLE == 2)
#   define AVG_SHIFT 1                  ///< Average division factor
#elif (NUM_SAMPLE == 4)
#   define AVG_SHIFT 2                  ///< Average division factor
#elif (NUM_SAMPLE == 8)
#   define AVG_SHIFT 3                  ///< Average division factor
#elif (NUM_SAMPLE == 16)
#   define AVG_SHIFT 4                  ///< Average division factor
#else
#   error Sample number is too large
#endif

/*---------------------------------- Macros ----------------------------------*/

/*----------------------------------- Const ----------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*---------------------------------- Locals ----------------------------------*/

VAR_STATIC INT8U Battery_Sample[NUM_SAMPLE];    ///< Array of last voltage samples
VAR_STATIC INT8U Battery_Buffer[BATT_ADC_NUM];  ///< Buffer for ADC values
VAR_STATIC INT8U Battery_Value;                 ///< Averaged battery voltage
VAR_STATIC INT8U Cnt_Sample;                    ///< Number of samples for average

/*--------------------------------- Prototypes -------------------------------*/

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Initialize battery management.
/// \remarks    -
//
//-----------------------------------------------------------------------------
void Battery_Init(void)
{
  INT8U i;

  for (i = 0; i < NUM_SAMPLE; i++) {           // Initialize sample buffer to nominal values
    Battery_Sample[i] = NOMINAL_THR;
  }
  Battery_Value = NOMINAL_THR;                 // Initialize battery voltage to nominal value
  Cnt_Sample = 0;
  TPool_Start(TMR_BATTERY, TMR_50_ms);  // Start timer for battery voltage sampling
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Calculate average value of a buffer elements
/// \param      buf: address of buffer
/// \return     Averaged battery value
/// \remarks    -
//
//-----------------------------------------------------------------------------
static INT8U Average_Value(const INT8U* buf)
{
 INT8U i;
 INT16U avg = 0;

 for (i = 0; i < NUM_SAMPLE; i++) {
   avg += (INT16U)buf[i];
 }
 return ((INT8U)(avg >> AVG_SHIFT));
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Battery Manager
/// \remarks    Battery voltage is sampled by an externel ADC connected via I2C.
///             Voltage samples are averaged over last NUM_SAMPLE values.
//
//-----------------------------------------------------------------------------
void Battery_Manager(void)
{
  INT8U adc_val;

  Ctrl_Flow_Cnt += CALLED_BATTERY;

  if (TPool_End(TMR_BATTERY)) {                  // Sampling time elapsed
                                                        // Read external ADC via I2C
    if (TWI_Read(BATTERY_ADC_ADDR,                      // ADC slave address
                 BATT_START_ADDR,                       // register address
                 BATT_ADC_NUM,                          // size
                 Battery_Buffer) == BATT_ADC_NUM) {     // destination pointer
      adc_val  = (INT8U)((INT8U)(Battery_Buffer[0] & 0x0F) << 4);
      adc_val += (INT8U)((INT8U)(Battery_Buffer[1] & 0xF0) >> 4);
      Battery_Sample[Cnt_Sample++] = adc_val;           // Save ADC value into buffer
      Cnt_Sample &= MAX_SAMPLE;                         // Saturate counter of samples
      Battery_Value = Average_Value(Battery_Sample);    // Calculate average
      TPool_Start(TMR_BATTERY, TMR_100_ms);      // Restart sampling timer
    }
  }

  Ctrl_Flow_Cnt_Inv -= CALLED_BATTERY;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Read battery voltage
/// \return     Battery voltage
/// \remarks    -
//
//-----------------------------------------------------------------------------
INT8U Read_Battery(void)
{
  return Battery_Value;
}

