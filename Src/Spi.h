//============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/Spi.h $
// $Revision: 9742 $
// $Date: 2011-02-09 10:05:27 +0100 (mer, 09 feb 2011) $
// $Author: lorenzo $
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

#ifndef SPI_H
#define SPI_H

/*--------------------------------- Definitions ------------------------------*/

/*------------------------------------ Macros --------------------------------*/

/*--------------------------------- Enumerations  ----------------------------*/

/*------------------------------------ Types ---------------------------------*/

/*------------------------------- Global variables ---------------------------*/

/*---------------------------------- Interface -------------------------------*/

/// @defgroup SPI SPI driver
/// @{

void Spi_Init(void);                      //!< Initialize SPI driver
void RTX_Timeout(void);                   //!< Timeout for telegram requests from RTX
BOOLEAN8 Spi_Puts(const INT8U *, INT8U);  //!< Put data into SPI transmission buffer
INT8U Get_Timestamp(void);            //!< Get current telegram timestamp
INT8U Get_TLG_Mux(void);                  //!< Get current telegram multiplexer bit

/// @}

#endif
