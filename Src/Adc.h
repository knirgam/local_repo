//============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/Adc.h $
// $Revision: 9742 $
// $Date: 2011-02-09 10:05:27 +0100 (mer, 09 feb 2011) $
// $Author: lorenzo $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief Adc driver
///
/// \file
///
// CHANGES doxygen
//
//============================================================================

/*-------------------------------- Definitions ------------------------------*/

#ifndef ADC_H
#define ADC_H

#undef VAR_GLOBAL
#define VAR_GLOBAL extern

/*----------------------------------- Globals -------------------------------*/

/*---------------------------------- Interface ------------------------------*/

/// @defgroup ADC ADC driver
/// @{

INT8U Adc_Get_Aux9(void);       ///< Get value of auxiliary proportional command A9.
BOOLEAN8 Adc_Get_Cmd(INT8U *);  ///< Get value of main proportional commands A1 - A8.
void Adc_Init(void);            ///< ADC initialization function.
void Adc_Manager(void);         ///< ADC manager function.
BOOLEAN8 Is_Sampling(void);     ///< ADC sampling in progress.
void Start_Sampling(void);      ///< Activates sampling.

/// @}

#endif
