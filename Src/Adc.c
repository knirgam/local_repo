//============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/Adc.c $
// $Revision: 9742 $
// $Date: 2011-02-09 10:05:27 +0100 (mer, 09 feb 2011) $
// $Author: lorenzo $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief ADC driver module.
///
/// \file
/// ADC driver is normally idle waiting for a sampling request.\n\n
/// Upon a sampling request (Flg_Adc_On = TRUE), both internal ADC and external
/// ADC (ADC081C027) are started.\n\n
/// ADC driver then waits for internal ADC to end conversion of proportional
/// commands.\n\n
/// Auxiliary proportional command A9 is read at most every two sampling of
/// main proportional commands: consecutive calls to TWI_Read() start external
/// ADC or actually read conversion result.
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
#include "TWI_Master.h"
#include "Adc.h"

/*--------------------------------- Definitions ------------------------------*/

#undef  VAR_STATIC
#define VAR_STATIC static

#define ADC_INPUTS     8            ///< Number of channels of internal ADC
#define MAX_ADC_VAL    (INT16U)1020 ///< Upper limit for internal ADC value
#define AUX_START_ADDR (INT16U)0    ///< Starting address of external ADC
#define AUX_ADC_NUM    (INT8U)2     ///< Number of bytes read from external ADC
#define AUX_ADC_ADDR   (INT8U)0x51  ///< I2C slave address of external ADC

#if   (ADC_INPUTS == 2)
#   define ADC_MAX_INDEX 1          ///< Maximum index of internal ADC sample buffer
#elif (ADC_INPUTS == 4)
#   define ADC_MAX_INDEX 3          ///< Maximum index of internal ADC sample buffer
#elif (ADC_INPUTS == 8)
#   define ADC_MAX_INDEX 7          ///< Maximum index of internal ADC sample buffer
#else
#   error ADC input number is not a power of 2
#endif

/*----------------------------------- Macros ----------------------------------*/

#define ADMUX_INIT()       (ADMUX = 0x00) ///< Initialization of internal ADC multiplexer register
                                          /* 000XXXXX
                                             ||||||||
                                             |||||||+- MUX0
                                             ||||||+-- MUX1
                                             |||||+--- MUX2
                                             ||||+---- MUX3
                                             |||+----- Reserved
                                             ||+------ ADLAR   ADC Left Adjust Result (0 -> right)
                                             |+------- REFS0   Voltage reference selection
                                             +-------- REFS1   Voltage reference selection */
#define ADCSRA_INIT()      (ADCSRA = 0x16) ///< Initialization of internal ADC control and status register A
                                           /* 00010110
                                              ||||||||
                                              |||||||+- ADPS0  ADC Prescaler Select Bit 1
                                              ||||||+-- ADPS1  ADC Prescaler Select Bit 2
                                              |||||+--- ADPS2  ADC Prescaler Select Bit 3
                                              ||||+---- ADIE   ADC Interrupt Enable
                                              |||+----- ADIF   ADC Interrupt Flag
                                              ||+------ ADATE  ADC Auto Trigger Enable
                                              |+------- ADSC   ADC Start Conversion
                                              +-------- ADEN   ADC Enable */
#define ADCSRB_INIT()      (ADCSRB = 0x00) ///< Initialization of internal ADC control and status register B
                                           /* XXXXX000
                                              ||||||||
                                              |||||||+- ADTS0  ADC Auto Trigger Source Selections Bit 1
                                              ||||||+-- ADTS1  ADC Auto Trigger Source Selections Bit 2
                                              |||||+--- ADTS2  ADC Auto Trigger Source Selections Bit 3
                                              ||||+---- Reserved
                                              |||+----- Reserved
                                              ||+------ Reserved
                                              |+------- Reserved
                                              +-------- Reserved */

#define ADC_ENABLE()       (ADCSRA |= (1<<ADEN))  ///< Enable internal ADC.
#define ADC_DISABLE()      (ADCSRA &= ~(1<<ADEN)) ///< Disable internal ADC.
#define ADC_START()        (ADCSRA |= (1<<ADSC))  ///< Start conversion of internal ADC.
#define ADC_CLEAR_FLAG()   (ADCSRA |= (1<<ADIF))  ///< Clear interrupt flag of internal ADC.
#define ADC_ENABLE_INT()   (ADCSRA |= (1<<ADIE))  ///< Enable interrupt of internal ADC.
#define ADC_BUFFER_EMPTY() (Adc_Read_Index == Adc_Write_Index)  ///< Buffer of internal ADC is empty

/*------------------------------------ Const ---------------------------------*/

/*------------------------------------ Enums ---------------------------------*/

//! States of ADC Manager.
enum {
    ADC_IDLE,               ///< ADC Manager is idle
    ADC_WAIT_SAMPLES        ///< ADC Manager is waiting samples
};

/*----------------------------------- Globals --------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC INT8U Adc_Samples[ADC_INPUTS];  ///< Buffer for internal ADC samples
VAR_STATIC INT8U Aux_Adc_Buf[AUX_ADC_NUM]; ///< Buffer for external ADC data
VAR_STATIC INT8U Adc_Write_Index;          ///< Write index of internal ADC buffer
VAR_STATIC INT8U Adc_Read_Index;           ///< Read index of internal ADC buffer
VAR_STATIC BOOLEAN8 Flg_Adc_On;            ///< ADC sampling enabled
VAR_STATIC INT8U Adc_State;                ///< ADC driver state machine variable
VAR_STATIC INT8U Aux_Adc_Value;            ///< Sample of external ADC

/*---------------------------------- Prototypes -------------------------------*/

//-----------------------------------------------------------------------------
//
//  DESCRIPTION ADC initialization
/// \remarks    ADC inputs are shared with JTAG pins. JTAG cell must be disabled
///             during normal working (DEBUG_JTAG not defined) to avoid erroneous
///             input values on internal ADC.
//
//-----------------------------------------------------------------------------
void Adc_Init(void)
{
  DDRG &= ~0x02;        // JTAG enable pin (PG.1) as input
  ADMUX_INIT();         // Voltage reference = AREF, right adjust result
  ADCSRA_INIT();        // Auto trigger, clear interrupt flag, divide clock by 128
  ADCSRB_INIT();        // Free running mode
  ADC_CLEAR_FLAG();     // Clear pending interrupt flag
  ADC_ENABLE_INT();     // Enable ADC interrupt
  DDRF = 0x00;          // Configure analog port as input
#ifndef DEBUG_JTAG
  MCUCSR |= (1 << JTD); // Disable JTAG
  MCUCSR |= (1 << JTD); // Must be repeated twice
#endif
  Flg_Adc_On = FALSE;   // Disable ADC sampling
  Aux_Adc_Value = 0;    // Clear sample of external ADC
  Adc_Read_Index = 0;   // Initialize read index of ADC buffer
  Adc_Write_Index = 0;  // Initialize write index of ADC buffer
  Adc_State = ADC_IDLE; // Initialize ADC state machine
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION ADC Manager
/// \remarks    -
//
//-----------------------------------------------------------------------------
void Adc_Manager(void)
{
  static INT8U twi_ans = 0;

  Ctrl_Flow_Cnt += CALLED_ADC;

  switch (Adc_State) {

    case ADC_WAIT_SAMPLES:
      if (Flg_Adc_On == FALSE) {        // End of internal ADC sampling
        if (twi_ans == AUX_ADC_NUM) {   // TWI manager has read external ADC
          Aux_Adc_Value = ((INT8U)(Aux_Adc_Buf[0] & 0x0F) << 4);
          Aux_Adc_Value += ((INT8U)(Aux_Adc_Buf[1] & 0xF0) >> 4);
        }
        Adc_State = ADC_IDLE;
      }
      break;

    case ADC_IDLE:
    default:
      if (Flg_Adc_On == TRUE) {         // New sampling started
        Adc_Read_Index = 0;             // Clear read index of ADC buffer
        Adc_Write_Index = 0;            // Clear write index of ADC buffer
        ADC_ENABLE();                   // Enable internal ADC
        ADC_START();                    // Start internal ADC conversion
        twi_ans = TWI_Read(AUX_ADC_ADDR, AUX_START_ADDR, AUX_ADC_NUM, Aux_Adc_Buf);
                                        // Start external ADC conversion
        Adc_State = ADC_WAIT_SAMPLES;   // Wait end of conversion
      }
      break;
  }

  Ctrl_Flow_Cnt_Inv -= CALLED_ADC;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION ADC sampling in progress.
/// \return     TRUE if sampling in progress, FALSE otherwise.
/// \remarks    -
//
//-----------------------------------------------------------------------------
BOOLEAN8 Is_Sampling(void)
{
  return (Flg_Adc_On);
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Start ADC sampling.
/// \remarks    Forces ADC manager to exit from IDLE state.
//
//-----------------------------------------------------------------------------
void Start_Sampling(void)
{
  Flg_Adc_On = TRUE;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Get values of internal ADC.
/// \param      p_ch pointer to return value
/// \return     TRUE if another ADC channel is available, FALSE otherwise.
/// \remarks    -
//
//-----------------------------------------------------------------------------
BOOLEAN8 Adc_Get_Cmd(INT8U * p_ch)
{
  BOOLEAN8 available = FALSE;               // Sample buffer empty

  if (!ADC_BUFFER_EMPTY()) {                // Sampling buffer not empty
    *p_ch = Adc_Samples[Adc_Read_Index++];  // Get next channel from ADC buffer
    Adc_Read_Index &= ADC_MAX_INDEX;        // Update buffer read index
    available = TRUE;                       // Read all ADC channels
  }
  return available;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Get value of external ADC
/// \return     A9 value
/// \remarks    -
//
//-----------------------------------------------------------------------------
INT8U Adc_Get_Aux9(void)
{
  return Aux_Adc_Value;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION ADC interrupt service routine
/// \remarks    ADC channels are sampled sequentially, until all channels have
///             been read. Conversion result is 10 bits wide, only the 8 most
///             significant bits are kept.
//
//-----------------------------------------------------------------------------
//lint --e{528} suppress "ADC_ISR() not referenced"
#pragma vector=ADC_vect
static __interrupt void ADC_ISR(void)
{
  INT8U adc_low, adc_high;
  INT16U adc_val;

  adc_low = ADCL;                           // Read ADC buffer (LSB first)
  adc_high = ADCH;
  adc_val = (((INT16U)adc_high << 8) + (INT16U)adc_low);
  if (adc_val >= MAX_ADC_VAL) {
    adc_val = 255;
  } else {
    adc_val >>= 2;
  }
  Adc_Samples[Adc_Write_Index++] = (INT8U)adc_val; // Keep 8 most significant bits
  Adc_Write_Index &= ADC_MAX_INDEX;         // Update circular buffer index
  ADMUX &= ~(0x0F);                         // Update ADC mux for channel selection
  ADMUX |= Adc_Write_Index;
  if (Adc_Write_Index == 0) {               // End of channel
    ADC_DISABLE();
    Flg_Adc_On = FALSE;
  } else {
    ADC_START();                            // Start ADC conversion
  }
}
