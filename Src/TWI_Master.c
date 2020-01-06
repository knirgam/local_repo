//============================================================================
//
// $HeadURL: http://10.16.0.19/svn/FSA/Coder/trunk/Firmware/ATMega64/Src/TWI_Master.c $
// $Revision: 9742 $
// $Date: 2011-02-09 10:05:27 +0100 (mer, 09 feb 2011) $
// $Author: lorenzo $
//
/// COMPILER: IAR C/C++ for AVR 4.21A/W32 (4.21.0.5)
///
/// \brief TWI driver.
///
/// \file
/// Supports multi master access to I2C devices.
///
/// \par COMMON RESOURCES:
/// TWI is a shared resource, being used for :
///   - reading UID code from external eeprom (module Uid.c)
///   - reading battery voltage from external ADC (modules Adc.c, Battery.c)
/// \par
/// Contentions are managed with a "first come first served" policy: if the
/// TWI driver is free (TWI_Status = IDLE) the first call to TWI_Read() or
/// TWI_Write() engages the TWI driver.\n\n
/// TWI driver will remain engaged until the read / write operation has been
/// completed or an error has occurred.\n\n
/// At the end of read / write operation, TWI_Read() / TWI_Write() return
/// the number of bytes read / written, or an error code.\n\n
/// As long as TWI driver is engaged, concurrent requests of read / write
/// operations from other modules are simply refused by TWI_Read() /
/// TWI_Write() functions.\n\n
/// To detect concurrent requests, the functions TWI_Read() and TWI_Write()
/// verify that the direction bit (read/write) and the slave address passed
/// by the caller function match with those of the operation in progress.\n\n
///
/// \par FUNCTION SIZE:
/// TWI_Manager() far exceeds allowed size of one page, due to the complexity
/// of the multi-master capability of I2C protocol.\n\n
/// The state-machine structure has been kept as simple as possible: small
/// states, few transitions, simple conditions.\n\n
/// Remarks are added to every line of code.
///
// CHANGES doxygen
//
//============================================================================

#include  <ioavr.h>
#include "includes.h"
#include "TPool.h"
#include "TWI_Master.h"

/*--------------------------------- Definitions ------------------------------*/

#undef  VAR_STATIC
#define VAR_STATIC static

/****************************************************************************
  TWI Status/Control register definitions
****************************************************************************/

#define TWI_BUFFER_SIZE     (INT8U)16    ///< TWI Buffer size.
#define TWI_TWBR            (INT8U)0x15  ///< TWI Bit rate register setting.

/****************************************************************************
  Bit and byte definitions
****************************************************************************/

#define TWI_DIR_READ        (INT8U)0    ///< Direction bit : read
#define TWI_DIR_WRITE       (INT8U)1    ///< Direction bit : write
#define TWI_READ_BIT        (INT8U)0    ///< Position for R/W bit in the "address byte"
#define TWI_ADR_BITS        (INT8U)1    ///< Position of the slave address bits in the "init byte"

/****************************************************************************
  TWI State codes
****************************************************************************/
// General TWI status codes
#define TWI_START           (INT8U)0x08  ///< START has been transmitted
#define TWI_REP_START       (INT8U)0x10  ///< Repeated START has been transmitted
#define TWI_ARB_LOST        (INT8U)0x38  ///< Arbitration lost

// TWI transmitter status codes
#define TWI_MTX_ADR_ACK     (INT8U)0x18  ///< SLA+W has been tramsmitted and ACK received
#define TWI_MTX_ADR_NACK    (INT8U)0x20  ///< SLA+W has been tramsmitted and NACK received
#define TWI_MTX_DATA_ACK    (INT8U)0x28  ///< Data byte has been tramsmitted and ACK received

// TWI receiver status codes
#define TWI_MRX_ADR_ACK     (INT8U)0x40  ///< SLA+R has been tramsmitted and ACK received
#define TWI_MRX_DATA_ACK    (INT8U)0x50  ///< Data byte has been received and ACK tramsmitted
#define TWI_MRX_DATA_NACK   (INT8U)0x58  ///< Data byte has been received and NACK tramsmitted

/*----------------------------------- Macros ----------------------------------*/

//! Reset TWI peripheral
#define IIC_RESET()   ( TWCR = (0<<TWEN)|(0<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC))
//                                 |         |                    |         |          |
//                                 |         |                    +---------+----------+- No Signal requests.
//                                 |         +------------------------------------------- Disable Interrupt.
//                                 +----------------------------------------------------- Switch off TWI-interface.

//! Initialize TWI peripheral
#define IIC_INIT()    ( TWCR = (1<<TWEN)|(0<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC))
//                                 |         |                    |         |          |
//                                 |         |                    +---------+----------+- No Signal requests.
//                                 |         +------------------------------------------- Disable Interrupt.
//                                 +----------------------------------------------------- Enable TWI-interface and release TWI pins.

//! Send a byte
#define IIC_SEND()    ( TWCR = (1<<TWEN)|(0<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC))
//                                 |         |         |
//                                 |         |         +--------------------------------- Clear the flag to send byte.
//                                 |         +------------------------------------------- Disable TWI Interupt.
//                                 +----------------------------------------------------- TWI Interface enabled.

//! Send a START sequence
#define IIC_START()   ( TWCR = (1<<TWEN)|(0<<TWIE)|(1<<TWINT)|(0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|(0<<TWWC))
//                                 |         |         |                    |
//                                 |         |         |                    +------------ Initiate a START condition.
//                                 |         |         +--------------------------------- Clear the flag.
//                                 |         +------------------------------------------- Disable TWI Interupt.
//                                 +----------------------------------------------------- TWI Interface enabled.

//! Receive a byte and send ACK
#define IIC_RECEIVE() ( TWCR = (1<<TWEN)|(0<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC))
//                                 |         |         |          |
//                                 |         |         |          +---------------------- Send ACK after reception.
//                                 |         |         +--------------------------------- Clear the flag to send byte.
//                                 |         +------------------------------------------- Disable TWI Interupt.
//                                 +----------------------------------------------------- TWI Interface enabled.

//! Receive a byte and send NACK
#define IIC_NACK()    ( TWCR = (1<<TWEN)|(0<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC))
//                                 |         |         |          |
//                                 |         |         |          +---------------------- Send NACK after reception.
//                                 |         |         +--------------------------------- Clear the flag to send byte.
//                                 |         +------------------------------------------- Disable TWI Interupt.
//                                 +----------------------------------------------------- TWI Interface enabled.

//! Stop TWI
#define IIC_STOP()    ( TWCR = (1<<TWEN)|(0<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(0<<TWWC))
//                                 |         |         |                               |
//                                 |         |         |                               +- Initiate a STOP condition.
//                                 |         |         +--------------------------------- Clear the flag.
//                                 |         +------------------------------------------- Disable TWI Interupt.
//                                 +----------------------------------------------------- TWI Interface enabled.

#define SDA_IDLE()    (((PIND & 0x02) != 0) ? 1 : 0)   ///< SDA line is idle (logic level 1)

/*------------------------------------ Enums ---------------------------------*/

//! TWI state machine states
enum {
    STOP,       ///< Stop TWI operation
    IDLE,       ///< TWI is ready for new operation
    INIT,       ///< Initialize TWI peripheral
    START,      ///< Send START sequence to I2C bus
    SLA_W,      ///< Send slave address + write bit request
    ADDR_H,     ///< Send high address of register to be read/written
    ADDR_L,     ///< Send low address of register to be read/written
    RESTART,    ///< Send RESTART sequence to I2C bus
    SLA_R,      ///< Send slave address + read bit request
    RX_DATA,    ///< Receive data from slave device
    TX_DATA,    ///< Transmit data to slave device
    ERROR,      ///< Error status
    BUS_RESET,  ///< Send I2C bus reset sequence
    BUS_RELEASE ///< Force I2C bus release
};

/*------------------------------------- Locals -------------------------------*/

VAR_STATIC INT8U TWI_Buffer[TWI_BUFFER_SIZE]; ///< TWI buffer
VAR_STATIC INT8U TWI_Msg_Size;              ///< Number of bytes to be transmitted / received
VAR_STATIC INT8U TWI_Index;                 ///< Index of byte to be transmitted / received
VAR_STATIC INT8U TWI_Slave;                 ///< Address of slave device
VAR_STATIC INT8U TWI_Direction;             ///< Direction of operation (read / write)
VAR_STATIC INT8U TWI_Address_H;             ///< Internal memory address - high byte
VAR_STATIC INT8U TWI_Address_L;             ///< Internal memory address - low byte
VAR_STATIC INT8U TWI_Status;                ///< TWI state machine variable

/*----------------------------------- Prototypes -----------------------------*/

//-----------------------------------------------------------------------------
//
//  DESCRIPTION TWI initialization
/// \remarks    Configures TWI peripheral for master operation and resets to
///             standby state.
//
//-----------------------------------------------------------------------------
void TWI_Master_Init(void)
{
  TWBR = TWI_TWBR;                      // Set bit rate register
  TWDR = 0xFF;                          // Set TWI data register to release SDA line
  IIC_INIT();                           // Enable TWI peripheral
  TWI_Status = INIT;                    // Init TWI state machine
  TPool_Start(TMR_TWI_RECOVERY, TMR_1_s); // Start recovery timer for bus error
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION TWI is busy transmitting.
/// \return     TRUE if TWI is busy, FALSE otherwise
/// \remarks    TWI ransceiver is busy if TWI interrupt is triggered
//
//-----------------------------------------------------------------------------
static INT8U TWI_Transceiver_Busy(void)
{
  return (INT8U)(TWCR & (1 << TWINT));
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Start a TWI reading operation
/// \param      slave I2C address of slave device
/// \param      address address of byte to be read
/// \param      size number of bytes to be read
/// \param      msg pointer to destination data buffer
/// \return     Number of bytes actually read
/// \remarks    -
//
//-----------------------------------------------------------------------------
INT8U TWI_Read(INT8U slave, INT16U address, INT8U size, INT8U * msg)
{
  INT8U temp;

  switch (TWI_Status) {
    case IDLE:                                          // TWI manager free
      if ((size > 0) && (size <= TWI_BUFFER_SIZE)) {    // Size fits TWI buffer
        TWI_Direction = TWI_DIR_READ;                   // Direction is READ
        TWI_Slave = (INT8U)(slave << TWI_ADR_BITS);     // Set slave address
        TWI_Address_H = (INT8U)(address >> 8);          // Set data address
        TWI_Address_L = (INT8U)(address & 0x00FF);
        TWI_Msg_Size = size;                            // Set number of data to receive
        TWI_Status = START;                             // Engage TWI manager
        IIC_START();                                    // Send START sequence
        temp = 0;                                       // No bytes read yet
        TPool_Start(TMR_TWI_TO, TMR_200_ms);            // Start timeout
      } else {                                          // Wrong size
        temp = TWI_ERROR;                               // Error
      }
      break;

    case STOP:                                          // Reading ended
      if ((TWI_Slave == (slave << TWI_ADR_BITS)) &&     // Slave address is correct
          (TWI_Direction == TWI_DIR_READ)) {            // Direction is correct
        for (temp = 0; temp < size; temp++) {           // Copy message
          msg[temp] = TWI_Buffer[temp];
        }
        TWI_Status = IDLE;                              // Free TWI manager
      } else if (TPool_End(TMR_TWI_TO)) {               // I2C engaged by another master for too long
        TWI_Status = IDLE;                              // Free TWI manager
        temp = 0;                                       // No bytes read
      } else {                                          //
        temp = 0;                                       // No bytes read
      }
      break;

    case ERROR:                                         // Error during read
      if ((TWI_Slave == (slave << TWI_ADR_BITS)) &&     // Slave address is correct
          (TWI_Direction == TWI_DIR_READ)) {            // Direction is correct
        temp = TWI_ERROR;                               // Error code
        IIC_RESET();                                    // Release bus
        IIC_START();                                    // Send START sequence
        TPool_Start(TMR_TWI_TO, TMR_200_ms);            // Start timeout
        TWI_Status = BUS_RESET;                         // Reset I2C and free TWI manager
      } else if (TPool_End(TMR_TWI_TO)) {               // I2C engaged by another master for too long
        temp = 0;                                       // No bytes read
        IIC_RESET();                                    // Release bus
        IIC_START();                                    // Send START sequence
        TPool_Start(TMR_TWI_TO, TMR_200_ms);            // Start timeout
        TWI_Status = BUS_RESET;                         // Reset I2C and free TWI manager
      } else {                                          //
        temp = 0;                                       // No bytes read
      }
      break;

    case INIT:                                          // Initialization
      temp = 0;                                         // No bytes read
      break;

    default:                                            // TWI manager busy
      if (TPool_End(TMR_TWI_TO)) {                      // Timeout expired
        TWI_Status = ERROR;                             // Stop TWI and signal error condition
      }
      temp = 0;                                         // No bytes read
      break;
  }
  return temp;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION Start a TWI writing operation
/// \param      slave I2C address of slave device
/// \param      address address of byte to be written
/// \param      size number of bytes to be written
/// \param      msg pointer to sorce data buffer
/// \return     Number of bytes actually written
/// \remarks    -
//
//-----------------------------------------------------------------------------
INT8U TWI_Write(INT8U slave, INT16U address, INT8U size, const INT8U * msg)
{
  INT8U temp;

  switch (TWI_Status) {
    case IDLE:                                          // TWI manager free
      if ((size > 0) && (size <= TWI_BUFFER_SIZE)) {    // Size fits TWI buffer
        for (temp = 0; temp < size; temp++) {           // Copy message
          TWI_Buffer[temp] = msg[temp];
        }
        TWI_Direction = TWI_DIR_WRITE;                  // Direction is WRITE
        TWI_Slave = (INT8U)(slave << TWI_ADR_BITS);     // Set slave address
        TWI_Address_H = (INT8U)(address >> 8);          // Set data address
        TWI_Address_L = (INT8U)(address & 0x00FF);
        TWI_Msg_Size = size;                            // Set number of data to write
        TWI_Status = START;                             // Engage TWI manager
        IIC_START();                                    // Send START sequence
        temp = 0;                                       // No bytes written
        TPool_Start(TMR_TWI_TO, TMR_200_ms);            // Start timeout
      } else {                                          // Wrong size
        temp = TWI_ERROR;                               // Error
      }
      break;

    case STOP:                                          // Writing ended
      if ((TWI_Slave == (slave << TWI_ADR_BITS)) &&     // Slave address is correct
          (TWI_Direction == TWI_DIR_WRITE)) {           // Direction is correct
        temp = size;                                    // Number of bytes written
        TWI_Status = IDLE;                              // Free TWI manager
      } else if (TPool_End(TMR_TWI_TO)) {               // TWI engaged by another master for too long
        TWI_Status = IDLE;                              // Free TWI manager
        temp = 0;                                       // No bytes written
      } else {                                          //
        temp = 0;                                       // No bytes written
      }
      break;

    case ERROR:                                         // Error during write
      if ((TWI_Slave == (slave << TWI_ADR_BITS)) &&     // Slave address is correct
          (TWI_Direction == TWI_DIR_WRITE)) {           // Direction is correct
        temp = TWI_ERROR;                               // Error code
        IIC_RESET();                                    // Release bus
        IIC_START();                                    // Send START sequence
        TPool_Start(TMR_TWI_TO, TMR_200_ms);            // Start timeout
        TWI_Status = BUS_RESET;                         // Free TWI manager
      } else if (TPool_End(TMR_TWI_TO)) {               // TWI engaged by another master for too long
        temp = 0;                                       // No bytes written
        IIC_RESET();                                    // Release bus
        IIC_START();                                    // Send START sequence
        TPool_Start(TMR_TWI_TO, TMR_200_ms);            // Start timeout
        TWI_Status = BUS_RESET;                         // Free TWI manager
      } else {                                          //
        temp = 0;                                       // No bytes written
      }
      break;

    case INIT:                                          // Initialization
      temp = 0;                                         // No bytes written
      break;

    default:                                            // TWI manager busy
      if (TPool_End(TMR_TWI_TO)) {                      // timeout elapsed
        TWI_Status = ERROR;                             // Stop and signal error
      }
      temp = 0;                                         // No bytes written
      break;
  }
  return temp;
}

//-----------------------------------------------------------------------------
//
//  DESCRIPTION State machine for TWI reading and writing management
/// \return     -
/// \remarks    The state machine is started by functions TWI_Read() or
///             TWI_Write() and advances whenever a TWI event has occurred.
//
//-----------------------------------------------------------------------------
void TWI_Manager(void)
{
  Ctrl_Flow_Cnt += CALLED_TWI;

  switch (TWI_Status) {

    case INIT:                                          // Initializing
      if (TWI_Transceiver_Busy() != 0) {                // TWI busy
        break;                                          // Exit
      }                                                 // TWI free
      TWI_Status = IDLE;                                // Free TWI manager
      break;

     case IDLE:                                         // TWI manager free
      if (SDA_IDLE()) {                                 // SDA line is idle
        TPool_Start(TMR_TWI_RECOVERY, TMR_1_s);         // Re-start timeout for I2C recovery
      } else if (TPool_End(TMR_TWI_RECOVERY) ) {        // Recovery timeout elapsed
        IIC_RESET();                                    // Release bus
        IIC_START();                                    // Send START sequence
        TPool_Start(TMR_TWI_TO, TMR_200_ms);            // Start time-out for TWI communication
        TPool_Start(TMR_TWI_RECOVERY, TMR_1_s);         // Re-start timeout for I2C recovery
        TWI_Status = BUS_RESET;                         // Free TWI manager
      }
      break;

    case BUS_RESET:                                     // Reset bus error condition
      if (TWI_Transceiver_Busy() == 0) {                // TWI free
        break;                                          // Exit
      }                                                 // TWI busy
      switch (TWSR) {
        case TWI_START:                                 // START sequence successfull
        case TWI_REP_START:                             // Repeated START sequence successfull
          TWDR = 0xFF;                                  // Force SDA line to idle
          IIC_SEND();                                   // Force sending a byte
          break;

        default:
          break;
      }
      TWI_Status = BUS_RELEASE;                         // Go to bus release state
      break;

    case BUS_RELEASE:                                   // Release bus after reset procedure
      if (TWI_Transceiver_Busy() == 0) {                // TWI free
        break;                                          // Exit
      }                                                 // TWI busy
      IIC_STOP();                                       // Send Stop Bit
      TWI_Status = IDLE;                                // Return to IDLE state
      break;

    case START:                                         // Start
      if (TWI_Transceiver_Busy() == 0) {                // TWI free
        break;                                          // Exit
      }                                                 // TWI busy
      switch (TWSR) {
        case TWI_START:                                 // START sequence successfull
        case TWI_REP_START:                             // Repeated START sequence successfull
          TWDR = TWI_Slave & ((INT8U) ~(1 << TWI_READ_BIT)); // Send SLA + W
          IIC_SEND();                                   // Start sending
          TWI_Index = 0;                                // Clear TWI buffer index
          TWI_Status = SLA_W;                           // Next status
          break;
        default:                                        // Error
          TWI_Status = ERROR;                           // Stop
          break;
      }
      break;

    case SLA_W:                                         // SLA + W
      if (TWI_Transceiver_Busy() == 0) {                // TWI free
        break;                                          // Exit
      }                                                 // TWI busy
      switch (TWSR) {
        case TWI_MTX_ADR_ACK:                           // SLA + W successfull
          TWDR = TWI_Address_H;                         // Send ADDR H
          IIC_SEND();                                   // Start sending
          TWI_Status = ADDR_H;                          // Next status
          break;
        case TWI_MTX_ADR_NACK:                          // Error sending SLA + W
          IIC_START();                                  // Send START sequence
          TWI_Index = 0;                                // Clear TWI buffer index
          TWI_Status = START;                           // Return to START status
          break;
        case TWI_ARB_LOST:                              // Arbitration lost
          IIC_START();                                  // Send START sequence
          TWI_Status = START;                           // Return to START status
          break;
        default:                                        // Error
          TWI_Status = ERROR;                           // Stop
          break;
      }
      break;

    case ADDR_H:                                        // ADDR H
      if (TWI_Transceiver_Busy() == 0) {                // TWI free
        break;                                          // Exit
      }                                                 // TWI busy
      switch (TWSR) {
        case TWI_MTX_DATA_ACK:                          // ADDR H successfull
          TWDR = TWI_Address_L;                         // Send ADDR L
          IIC_SEND();                                   // Start sending
          TWI_Status = ADDR_L;                          // Next status
          break;
        case TWI_ARB_LOST:                              // Arbitration lost
          IIC_START();                                  // Send START sequence
          TWI_Status = START;                           // Return to START status
          break;
        default:                                        // NACK received or error
          TWI_Status = ERROR;                           // Stop
          break;
      }
      break;

    case ADDR_L:                                        // ADDR L
      if (TWI_Transceiver_Busy() == 0) {                // TWI free
        break;                                          // Exit
      }                                                 // TWI busy
      switch (TWSR) {
        case TWI_MTX_DATA_ACK:                          // ADDR L successfull
          if (TWI_Direction == TWI_DIR_READ) {          // Must read from slave
            IIC_START();                                // Send START sequence
            TWI_Status = RESTART;                       // Go to restart
          } else {                                      // Must write to target
            TWI_Status = TX_DATA;                       // Go to transmit data
          }
          break;
        case TWI_ARB_LOST:                              // Arbitration lost
          IIC_START();                                  // Send START sequence
          TWI_Status = START;                           // Go back to start
          break;
        default:                                        // NACK received or error
          TWI_Status = ERROR;                           // Stop
          break;
      }
      break;

    case RESTART:                                       // Repeated START
      if (TWI_Transceiver_Busy() == 0) {                // TWI free
        break;                                          // Exit
      }                                                 // TWI busy
      switch (TWSR) {
        case TWI_REP_START:                             // Repeated START successfull
          TWDR = TWI_Slave | (1 << TWI_READ_BIT);       // Send SLA + R
          TWI_Status = SLA_R;                           // Next status
          IIC_SEND();                                   // Start sending
          break;
        default:                                        // Error
          TWI_Status = ERROR;                           // Stop
          break;
      }
      break;

    case SLA_R:                                         // SLA + R
      if (TWI_Transceiver_Busy() == 0) {                // TWI free
        break;                                          // Exit
      }                                                 // TWI busy
      switch (TWSR) {
        case TWI_MRX_ADR_ACK:                           // SLA + R successfull
          if (TWI_Index < (TWI_Msg_Size - 1)) {         // Last byte not yet received
            IIC_RECEIVE();                              // Receive byte
          } else {                                      // Last byte received
            IIC_NACK();                                 // Send NACK after reception
          }
          TWI_Status = RX_DATA;                         // Next status
          break;
        default:                                        // Error
          TWI_Status = ERROR;                           // Stop
          break;
      }
      break;

    case RX_DATA:                                       // Receive data
      if (TWI_Transceiver_Busy() == 0) {                // TWI free
        break;                                          // Exit
      }                                                 // TWI busy
      switch (TWSR) {
        case TWI_MRX_DATA_ACK:                          // Data received and ACK sent
          TWI_Buffer[TWI_Index++] = TWDR;               // Save received data
          if (TWI_Index < (TWI_Msg_Size - 1)) {         // Last byte not yet received
            IIC_RECEIVE();                              // Receive byte
          } else {                                      // Last byte received
            IIC_NACK();                                 // Send NACK after reception
          }
          break;
        case TWI_MRX_DATA_NACK:                         // Data received and NACK sent
          TWI_Buffer[TWI_Index++] = TWDR;               // Save received data
          IIC_STOP();                                   // Send stop bit
          TWI_Status = STOP;                            // Stop TWI manager
          break;
        default:                                        // Error
          TWI_Status = ERROR;                           // Stop
          break;
      }
      break;

    case TX_DATA:                                       // Transmit data
      if (TWI_Transceiver_Busy() == 0) {                // TWI free
        break;                                          // Exit
      }                                                 // TWI busy
      switch (TWSR) {
        case TWI_MTX_DATA_ACK:                          // Data transmitted and ACK received
          if (TWI_Index < (TWI_Msg_Size)) {             // Last byte not yet transmitted
            TWDR = TWI_Buffer[TWI_Index++];             // Copy byte into transmite register
            IIC_SEND();                                 // Send data
          } else {                                      // Last byte transmitted
            IIC_STOP();                                 // Send stop bit
            TWI_Status = STOP;                          // Stop TWI manager
          }
          break;
        case TWI_ARB_LOST:                              // Arbitration lost
          IIC_START();                                  // Send START sequence
          TWI_Status = START;                           // Go back to start
          break;
        default:                                        // NACK received or error
          TWI_Status = ERROR;                           // Stop
          break;
      }
      break;

    default:
      break;
  }

  Ctrl_Flow_Cnt_Inv -= CALLED_TWI;
}
