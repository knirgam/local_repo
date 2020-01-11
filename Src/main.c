//============================================================================+
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/main.c $
// $Revision: 9861 $
// $Date: 2011-03-08 14:59:06 +0100 (mar, 08 mar 2011) $
// $Author: micheleb $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)

/// Flow control variables \c Ctrl_Flow_Cnt_Inv and \c Ctrl_Flow_Cnt need to be
/// global in order to be visible by all modules.
///
//  CHANGES Removed call to Stop_Pulse_Off()
//
//============================================================================*/

#include  <ioavr.h>
#include  <intrinsics.h>
#include "includes.h"
#include "Crc.h"
#include "Adc.h"
#include "Digital.h"
#include "Uart0.h"
#include "Uart1.h"
#include "Spi.h"
#include "Stop_Monitor.h"
#include "Code.h"
#include "TPool.h"
#include "Uid.h"adsf
#include "Battery.h"
#include "TWI_Master.h"
#include "Hic.h"
#include "Coder_Coder_A.h"
#include "Logic.h"


Modifiche sul branch master
/*--------------------------------- Definitions ------------------------------*/
Isolved the conglict
#define CONTROL_FLOW_OK (CALL_RTX       + CALLED_RTX       + \
                         CALL_STATEFLOW + \
                         CALL_ADC       + CALLED_ADC       + \
                         CALL_DIGITAL   + CALLED_DIGITAL   + \
                         CALL_UART1     + CALLED_UART1     + \
                         CALL_HIC       + CALLED_HIC       + \
                         CALL_STOP      + CALLED_STOP      + \
                         CALL_BATTERY   + CALLED_BATTERY   + \
                         CALL_TWI       + CALLED_TWI       + \
                         CALL_ROM       + CALLED_ROM       )

#define PROGRAM_FLOW_ERR    ((INT8U) 0xEF) //!< Program flow error code

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

//
// Flow control counter, critical variable
//
__no_init VAR_GLOBAL INT16U Ctrl_Flow_Cnt     @ "SAFETY_RAM";
__no_init VAR_GLOBAL INT16U Ctrl_Flow_Cnt_Inv @ "SAFETY_RAM_REV";

/*----------------------------------- Locals ---------------------------------*/

/*--------------------------------- Prototypes -------------------------------*/

///----------------------------------------------------------------------------
///
/// DESCRIPTION  Main function
/// \remarks     Initialize modules with disabled interrupts, then enable interrupts
///              and enter an infite loop where execute continuously modules manager
///              functions. Before and after the execution of each manager function
///              the control flow variable and its complementary value are increment
///              or decrement. Prime numbers are used. At the end of every loop the sum
///              of control flow variable is checked in order to detect errors in the
///              execution flow of the program.
///
///----------------------------------------------------------------------------
void main(void)
{
  INT8U failure;

  PORTA &= ~(1 << PA0);              // Auto-test finished: turn on internal green LED
  Led_Red_Off();                     // Turn off internal red LED
  DDRA |= (1 << PA0) + (1 << PA1);
  TPool_Init();                      // Init software timers
  Uart0_Init();                      // Init Uart0
  Uart1_Init();                      // Init Uart1
  Digital_Init();                    // Init digital inputs driver
  Adc_Init();                        // Init ADC driver
  Hic_Init();                        // Init Hic interface
  Code_Init();                       // Init coding
  Spi_Init();                        // Init SPI driver
  Stop_Monitor_Init();               // Init STOP monitor
  Uid_Init();                        // Init UID manager
  Battery_Init();                    // Init battery management
  TWI_Master_Init();                 // Init TWI/I2C driver
  Ctrl_Flow_Cnt = 0;                 // Init flow control variable
  Ctrl_Flow_Cnt_Inv = 0xFFFF;        // Init complemented flow control variable
  TPool_Start(TMR_STOP_PULSE, TMR_94_ms); // Start timer for STOP pulse generation
  Coder_initializer();               // Init coding state machine
  Crc16_Flash_Init();                // Initializes CRC check of Flash memory
  __enable_interrupt();              // Enable General interrupt flag

  for (; ; ) {

    Ctrl_Flow_Cnt += CALL_RTX;
    RTX_Timeout();                    // RTX monitor
    Ctrl_Flow_Cnt_Inv -= CALL_RTX;

    Ctrl_Flow_Cnt += CALL_STATEFLOW;
    broadcast_Logic_Ev_Tick();        // Broadcast TICK event to Stateflow executive
    Ctrl_Flow_Cnt_Inv -= CALL_STATEFLOW;

    Ctrl_Flow_Cnt += CALL_ADC;
    Adc_Manager();                    // ADC
    Ctrl_Flow_Cnt_Inv -= CALL_ADC;

    Ctrl_Flow_Cnt += CALL_DIGITAL;
    Digital_Manager();                // Digital inputs
    Ctrl_Flow_Cnt_Inv -= CALL_DIGITAL;

    Ctrl_Flow_Cnt += CALL_UART1;
    Uart1_Manager();                  // UART 1
    Ctrl_Flow_Cnt_Inv -= CALL_UART1;

    Ctrl_Flow_Cnt += CALL_HIC;
    Hic_Manager();                    // Aux Commands from Hic
    Ctrl_Flow_Cnt_Inv -= CALL_HIC;

    Ctrl_Flow_Cnt += CALL_STOP;
    Stop_Monitor_Manager();           // Stop monitor
    Ctrl_Flow_Cnt_Inv -= CALL_STOP;

    Ctrl_Flow_Cnt += CALL_BATTERY;
    Battery_Manager();                // Battery monitor
    Ctrl_Flow_Cnt_Inv -= CALL_BATTERY;

    Ctrl_Flow_Cnt += CALL_TWI;
    TWI_Manager();                    // I2C
    Ctrl_Flow_Cnt_Inv -= CALL_TWI;

    Ctrl_Flow_Cnt += CALL_ROM;
    Crc16_Flash_Check();              // Program memory check
    Ctrl_Flow_Cnt_Inv -= CALL_ROM;

    if ((Ctrl_Flow_Cnt == CONTROL_FLOW_OK) &&             // Program flow OK
       ((Ctrl_Flow_Cnt ^ Ctrl_Flow_Cnt_Inv) == 0xFFFF)) {
      Ctrl_Flow_Cnt = 0;                                  // Reinitialize control flow variable
      Ctrl_Flow_Cnt_Inv = 0xFFFF;                         // Reinitialize complemented control flow variable
      if (TPool_End(TMR_STOP_PULSE)) {                    // Time for STOP pulse generation
        Stop_Pulse_On();                                  // Enable STOP pulse
        TPool_Start(TMR_STOP_PULSE, TMR_94_ms);           // Restart timer for STOP pulse generation
      }
    } else {                                              // Program flow drift

      /************************************************************************/
      /*    the following code is executed only if a critical error happened: */
      /*    no function call has to be executed because stack or registers    */
      /*    could be compromized                                              */
      /************************************************************************/
      __disable_interrupt();                // Disable all interrupts

      #pragma diag_suppress=Pm038           // Bypass MISRA pointer use inhibition

      /**************** read previously stored error code *********************/

      EEAR = (INT16U)&Saved_Failure;        // Set up address register
      EECR |= (1<<EERE);                    // Start eeprom read
      failure = EEDR;                       // Read data register

      /************************************************************************/

      failure &= PROGRAM_FLOW_ERR;          // Add new error code

      /**************** write error code into eeprom **************************/

      while ((EECR & (1<<EEWE)) != 0) {     // wait till eeprom is busy
        ;
      }
      EEAR = (INT16U)&Saved_Failure;        // Set up address register
      EEDR = failure;                       // Set up data register
      EECR |= (1<<EEMWE);                   // Enable eeprom write
      EECR |= (1<<EEWE);                    // Start eeprom write

      /************************************************************************/

      #pragma diag_error=Pm038

      DDRD  |= 0x40;                        // Set SHUTDOWN pin as output (PD.6)
      PORTD |= 0x40;                        // Shutdown RTX
      for (; ; ) {
      }
    }
  }
}



//-----------------------------------------------------------------------------
//
// DESCRIPTION INT0_vect
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Int0_ISR() not referenced"
#pragma vector=INT0_vect
static __interrupt void Int0_ISR (void)
{
}

//-----------------------------------------------------------------------------
//
// DESCRIPTION INT1_vect
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Int1_ISR() not referenced"
#pragma vector=INT1_vect
static __interrupt void Int1_ISR (void)
{
}

//-----------------------------------------------------------------------------
//
// DESCRIPTION INT2_vect
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Int2_ISR() not referenced"
#pragma vector=INT2_vect
static __interrupt void Int2_ISR (void)
{
}

//-----------------------------------------------------------------------------
//
// DESCRIPTION INT3_vect
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Int3_ISR() not referenced"
#pragma vector=INT3_vect
static __interrupt void Int3_ISR (void)
{
}

//-----------------------------------------------------------------------------
//
// DESCRIPTION INT5_vect
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Int5_ISR() not referenced"
#pragma vector=INT5_vect
static __interrupt void Int5_ISR (void)
{
}



//-----------------------------------------------------------------------------
//
// DESCRIPTION INT6_vect
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Int6_ISR() not referenced"
#pragma vector=INT6_vect
static __interrupt void Int6_ISR (void)
{
}


//-----------------------------------------------------------------------------
//
// DESCRIPTION INT7_vect
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Int7_ISR() not referenced"
#pragma vector=INT7_vect
static __interrupt void Int7_ISR (void)
{
}


//-----------------------------------------------------------------------------
//
// DESCRIPTION Timer2_Comp_ISR
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Timer2_Comp_ISR() not referenced"
#pragma vector=TIMER2_COMP_vect
static __interrupt void Timer2_Comp_ISR (void)
{
}

//-----------------------------------------------------------------------------
//
// DESCRIPTION Timer1_Capt_ISR
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Timer1_Capt_ISR() not referenced"
#pragma vector=TIMER1_CAPT_vect
static __interrupt void Timer1_Capt_ISR (void)
{
}

//-----------------------------------------------------------------------------
//
// DESCRIPTION Timer1_CompA_ISR
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Timer1_CompA_ISR() not referenced"
#pragma vector=TIMER1_COMPA_vect
static __interrupt void Timer1_CompA_ISR (void)
{
}


//-----------------------------------------------------------------------------
//
// DESCRIPTION Timer1_CompB_ISR
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Timer1_CompB_ISR() not referenced"
#pragma vector=TIMER1_COMPB_vect
static __interrupt void Timer1_CompB_ISR (void)
{
}

//-----------------------------------------------------------------------------
//
// DESCRIPTION Timer1_Ovf_ISR
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Timer1_Ovf_ISR() not referenced"
#pragma vector=TIMER1_OVF_vect
static __interrupt void Timer1_Ovf_ISR (void)
{
}


//-----------------------------------------------------------------------------
//
// DESCRIPTION Timer0_Comp_ISR
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Timer0_Comp_ISR() not referenced"
#pragma vector=TIMER0_COMP_vect
static __interrupt void Timer0_Comp_ISR (void)
{
}

//-----------------------------------------------------------------------------
//
// DESCRIPTION Timer0_Ovf_ISR
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Timer0_Ovf_ISR() not referenced"
#pragma vector=TIMER0_OVF_vect
static __interrupt void Timer0_Ovf_ISR (void)
{
}

//-----------------------------------------------------------------------------
//
// DESCRIPTION Usart0_Txc_ISR
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Usart0_Txc_ISR() not referenced"
#pragma vector=USART0_TXC_vect
static __interrupt void Usart0_Txc_ISR (void)
{
}


//-----------------------------------------------------------------------------
//
// DESCRIPTION EE_RDY_ISR
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "EE_RDY_ISR() not referenced"
#pragma vector=EE_RDY_vect
static __interrupt void EE_RDY_ISR (void)
{
}


//-----------------------------------------------------------------------------
//
// DESCRIPTION ANA_Comp_ISR
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "ANA_Comp_ISR() not referenced"
#pragma vector=ANA_COMP_vect
static __interrupt void ANA_Comp_ISR (void)
{
}

//-----------------------------------------------------------------------------
//
// DESCRIPTION Timer1_CompC_ISR
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Timer1_CompC_ISR() not referenced"
#pragma vector=TIMER1_COMPC_vect
static __interrupt void Timer1_CompC_ISR (void)
{
}


//-----------------------------------------------------------------------------
//
// DESCRIPTION Timer3_Capt_ISR
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Timer3_Capt_ISR() not referenced"
#pragma vector=TIMER3_CAPT_vect
static __interrupt void Timer3_Capt_ISR (void)
{
}


//-----------------------------------------------------------------------------
//
// DESCRIPTION Timer3_CompA_ISR
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Timer3_CompA_ISR() not referenced"
#pragma vector=TIMER3_COMPA_vect
static __interrupt void Timer3_CompA_ISR (void)
{
}


//-----------------------------------------------------------------------------
//
// DESCRIPTION Timer3_CompB_ISR
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Timer3_CompB_ISR() not referenced"
#pragma vector=TIMER3_COMPB_vect
static __interrupt void Timer3_CompB_ISR (void)
{
}

//-----------------------------------------------------------------------------
//
// DESCRIPTION Timer3_CompC_ISR
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Timer3_CompC_ISR() not referenced"
#pragma vector=TIMER3_COMPC_vect
static __interrupt void Timer3_CompC_ISR (void)
{
}



//-----------------------------------------------------------------------------
//
// DESCRIPTION Usart1_UDRE_ISR
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Usart1_UDRE_ISR() not referenced"
#pragma vector=USART1_UDRE_vect
static __interrupt void Usart1_UDRE_ISR (void)
{
}



//-----------------------------------------------------------------------------
//
// DESCRIPTION Usart1_Txc_ISR
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "Usart1_Txc_ISR() not referenced"
#pragma vector=USART1_TXC_vect
static __interrupt void Usart1_Txc_ISR (void)
{
}

//-----------------------------------------------------------------------------
//
// DESCRIPTION TWI_ISR
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "TWI_ISR() not referenced"
#pragma vector=TWI_vect
static __interrupt void TWI_ISR (void)
{
}


//-----------------------------------------------------------------------------
//
// DESCRIPTION SPM_RDY_ISR
/// \remarks
//  Not used
//-----------------------------------------------------------------------------
//lint --e{528} suppress "SPM_RDY_ISR() not referenced"
#pragma vector=SPM_RDY_vect
static __interrupt void SPM_RDY_ISR (void)
{
}







