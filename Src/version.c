//============================================================================+
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/version.c $
// $Revision: 10808 $
// $Date: 2011-11-08 11:32:31 +0100 (mar, 08 nov 2011) $
// $Author: lorenzo $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief Firmware revision module
///
/// \file
///
// CHANGES revision REV 02
//
//============================================================================*/
// changes for remote repo
// changes for local repo
// changes for local staged area
// changes not staged yet
#include "includes.h"
#include "version.h"

/*--------------------------------- Definitions ------------------------------*/

#undef VAR_STATIC
#define VAR_STATIC static

#define VERSION_STR_SIZE 16   ///< Length of revision string

/*---------------------------------- Constants -------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

/// Revision string
VAR_STATIC INT8U __flash Version_String[VERSION_STR_SIZE] = "CoderA_REV-02\r";

/*--------------------------------- Prototypes -------------------------------*/

//-----------------------------------------------------------------------------
//
// DESCRIPTION Read firmware version
/// \param     psz_str pointer to destination string
/// \param     size length of destination buffer
/// \remarks   -
//
//-----------------------------------------------------------------------------
void Version_Read(INT8U * psz_str, INT8U size)
{
   INT8U i, ch;
   BOOLEAN8 eol = FALSE;

   for (i = 0; (i < size) && (i < VERSION_STR_SIZE) && (eol == FALSE); i++) {
       ch = Version_String[i];
       *psz_str++ = ch;
       eol = ((ch == '\0') ? 1 : 0);
   }
}

