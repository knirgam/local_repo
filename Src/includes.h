//============================================================================+
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/includes.h $
// $Revision: 9742 $
// $Date: 2011-02-09 10:05:27 +0100 (mer, 09 feb 2011) $
// $Author: lorenzo $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief Global definitions module
///
/// \file
///
// CHANGES doxygen
//
//============================================================================*/

/*-------------------------------- Definitions -------------------------------*/

#ifndef INCLUDES_H
#   define INCLUDES_H

#   ifndef TRUE
#       define TRUE (1)         ///< Boolean TRUE
#   endif
#   ifndef FALSE
#       define FALSE (0)        ///< Boolean FALSE
#   endif

#   ifndef true
#       define true (1)         ///< Boolean true
#   endif
#   ifndef false
#       define false (0)        ///< Boolean false
#   endif

#   ifdef VAR_GLOBAL
#       undef VAR_GLOBAL
#   endif
#   define VAR_GLOBAL extern

//
// Prime numbers for flow control: called functions
//
#   define CALLED_RTX       3  ///< RTX monitor
#   define CALLED_STATEFLOW 7  ///< Stateflow
#   define CALLED_ADC      13  ///< ADC
#   define CALLED_DIGITAL  19  ///< Digital
#   define CALLED_UART1    29  ///< UART 1
#   define CALLED_HIC      37  ///< Hic
#   define CALLED_STOP     43  ///< Stop monitor
#   define CALLED_BATTERY  53  ///< Battery
#   define CALLED_TWI      61  ///< TWI
#   define CALLED_ROM      71  ///< Flash


/*------------------------------------ Types ---------------------------------*/

// typedef for generic data types
typedef signed char INT8;       ///< Type for 8 bit signed char
typedef unsigned char INT8U;    ///< Type for 8 bit unsigned char
typedef signed int INT16;       ///< Type for 16 bit signed word
typedef unsigned int INT16U;    ///< Type for 16 bit unsigned word
typedef signed long INT32;      ///< Type for 32 bit signed double word
typedef unsigned long INT32U;   ///< Type for 32 bit unsigned double word
typedef unsigned char BOOLEAN8; ///< Type for 8 bit long boolean

/*---------------------------------- Globals ---------------------------------*/

//
// Flow control variables are global in order to be visible by all modules
//
VAR_GLOBAL INT16U Ctrl_Flow_Cnt;        ///< Control flow counter
VAR_GLOBAL INT16U Ctrl_Flow_Cnt_Inv;    ///< Inverted control flow counter

///! Critical error codes
VAR_GLOBAL __no_init volatile __eeprom INT8U Saved_Failure;

#endif

