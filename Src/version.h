//============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/version.h $
// $Revision: 9742 $
// $Date: 2011-02-09 10:05:27 +0100 (mer, 09 feb 2011) $
// $Author: lorenzo $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief  Version header file
///
/// \file
///
// CHANGES doxygen
//
//============================================================================

#ifndef VERSION_H
#   define VERSION_H

/*------------------------------- Definitions -------------------------------*/

/*--------------------------------- Globals --------------------------------*/

/*-------------------------------- Interface --------------------------------*/

/// @defgroup VERSION Firmware revision
/// @{

void Version_Read(INT8U*, INT8U);   ///< Read firmware revision
/// @}

#endif
