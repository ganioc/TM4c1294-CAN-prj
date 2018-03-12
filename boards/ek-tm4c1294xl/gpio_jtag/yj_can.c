#include "drivers/yj_can.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_can.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/can.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h"

//*****************************************************************************
//
// A counter that keeps track of the number of times the TX interrupt has
// occurred, which should match the number of TX messages that were sent.
//
//*****************************************************************************
volatile uint32_t g_ui32MsgCount = 0;
volatile uint32_t g_ui32MsgTxCount = 0;
volatile uint32_t g_ui32MsgRxCount = 0;
extern uint32_t g_ui32SysClock;

//*****************************************************************************
//
// A flag to indicate that some transmission error occurred.
//
//*****************************************************************************
volatile bool g_bErrFlag = 0;




tCANMsgObject sCANMessageTx;
uint8_t pui8MsgDataTx[8];
uint8_t pui8MsgDataTxLen;

tCANMsgObject sCANMessageRx;
uint8_t pui8MsgDataRx[8];
uint8_t pui8MsgDataRxLen;

//*****************************************************************************
//
// This function provides a 1 second delay using a simple polling method.
//
//*****************************************************************************
void
SimpleDelay(void)
{
    //
    // Delay cycles for 1 second
    //
    SysCtlDelay(g_ui32SysClock/3);
}
//*****************************************************************************
//
// A flag for the interrupt handler to indicate that a message was received.
//
//*****************************************************************************
volatile bool g_bRXFlag = 0;

//*****************************************************************************
//
// This function is the interrupt handler for the CAN peripheral.  It checks
// for the cause of the interrupt, and maintains a count of all messages that
// have been transmitted.
//
//*****************************************************************************
void
CANIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(ui32Status == CAN_INT_INTID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.  If the
        // CAN peripheral is not connected to a CAN bus with other CAN devices
        // present, then errors will occur and will be indicated in the
        // controller status.
        //
        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        //
        // Set a flag to indicate some errors may have occurred.
        //
        g_bErrFlag = 1;
    }

    //
    // Check if the cause is message object 1, which what we are using for
    // sending messages.
    //
    else if(ui32Status == MSG_OBJ_TX_NUM)
    {
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 1, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, MSG_OBJ_TX_NUM);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ui32MsgTxCount++;
        
        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlag = 0;
    }
    else if(ui32Status == MSG_OBJ_RX_NUM)
    {
        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message reception is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, MSG_OBJ_RX_NUM);

        //
        // Increment a counter to keep track of how many messages have been
        // received.  In a real application this could be used to set flags to
        // indicate when a message is received.
        //
        g_ui32MsgRxCount++;

        //
        // Set flag to indicate received message is pending.
        //
        g_bRXFlag = 1;

        //
        // Since a message was received, clear any error flags.
        //
        g_bErrFlag = 0;
    }
    //
    // Otherwise, something unexpected caused the interrupt.  This should
    // never happen.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }
}


void InitCAN(uint32_t datarate){
  
      // For this example CAN0 is used with RX and TX pins on port A0 and A1.
    // The actual port and pins used may be different on your part, consult
    // the data sheet for more information.
    // GPIO port B needs to be enabled so these pins can be used.
    // TODO: change this to whichever GPIO port you are using
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    // Configure the GPIO pin muxing to select CAN0 functions for these pins.
    // This step selects which alternate function is available for these pins.
    // This is necessary if your part supports GPIO pin function muxing.
    // Consult the data sheet to see which functions are allocated per pin.
    // TODO: change this to select the port/pin you are using
    //
    GPIOPinConfigure(GPIO_PA0_CAN0RX);
    GPIOPinConfigure(GPIO_PA1_CAN0TX);
    
    // Enable the alternate function on the GPIO pins.  The above step selects
    // which alternate function is available.  This step actually enables the
    // alternate function instead of GPIO for these pins.
    // TODO: change this to match the port/pin you are using
    //
    GPIOPinTypeCAN(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    // The GPIO port and pins have been set up for CAN.  The CAN peripheral
    // must be enabled.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
    
        //
    // Initialize the CAN controller
    //
    CANInit(CAN0_BASE);
    
        //
    // Set up the bit rate for the CAN bus.  This function sets up the CAN
    // bus timing for a nominal configuration.  You can achieve more control
    // over the CAN bus timing by using the function CANBitTimingSet() instead
    // of this one, if needed.
    // In this example, the CAN bus is set to 500 kHz.  In the function below,
    // the call to SysCtlClockGet() or ui32SysClock is used to determine the 
    // clock rate that is used for clocking the CAN peripheral.  This can be 
    // replaced with a  fixed value if you know the value of the system clock, 
    // saving the extra function call.  For some parts, the CAN peripheral is 
    // clocked by a fixed 8 MHz regardless of the system clock in which case 
    // the call to SysCtlClockGet() or ui32SysClock should be replaced with 
    // 8000000.  Consult the data sheet for more information about CAN 
    // peripheral clocking.
    
    CANBitRateSet(CAN0_BASE, g_ui32SysClock, datarate);
  
    
    // Enable interrupts on the CAN peripheral.  This example uses static
    // allocation of interrupt handlers which means the name of the handler
    // is in the vector table of startup code.  If you want to use dynamic
    // allocation of the vector table, then you must also call CANIntRegister()
    // here.
    //
    CANIntRegister(CAN0_BASE, CANIntHandler); // if using dynamic vectors
    //
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
        //
    // Enable the CAN interrupt on the processor (NVIC).
    //
    IntEnable(INT_CAN0);

    //
    // Enable the CAN for operation.
    //
    CANEnable(CAN0_BASE);
    
    
}

void InitCANTxMsg(uint32_t ID, uint8_t *buf, uint8_t len){
    // Initialize the message object that will be used for sending CAN
    // messages.  The message will be 4 bytes that will contain an incrementing
    // value.  Initially it will be set to 0.
    //
    // ui32MsgData = 0;
    uint8_t i;
    
    if(len > 8 || len <=0){
      UARTprintf("Wrong TX buf\n");
      return;
    }
    
    for(i=0; i< len ; i++){
      pui8MsgDataTx[i] = buf[i];
    }
    pui8MsgDataTxLen = len;
      
    
    sCANMessageTx.ui32MsgID = ID;
    sCANMessageTx.ui32MsgIDMask = 0;
    sCANMessageTx.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    sCANMessageTx.ui32MsgLen = pui8MsgDataTxLen; // sizeof(pui8MsgData);
    sCANMessageTx.pui8MsgData = pui8MsgDataTx;  
  
  
}
void InitCANRxMsg(uint32_t ID){
    sCANMessageRx.ui32MsgID = ID;
    sCANMessageRx.ui32MsgIDMask = 0;
    sCANMessageRx.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;;
    sCANMessageRx.ui32MsgLen = 8; // sizeof(pui8MsgData);
      
}
void ReceiveCANMsg(){
  
  UARTprintf("Receiving msg\n");
  
  CANMessageSet(CAN0_BASE, MSG_OBJ_RX_NUM, &sCANMessageRx, MSG_OBJ_TYPE_RX);
}
void SendCANMsg(){
  
  //
  // Print a message to the console showing the message count and the
  // contents of the message being sent.
  //
  
  UARTprintf("Sending msg: 0x%02X %02X %02X %02X",
             pui8MsgDataTx[0], pui8MsgDataTx[1], pui8MsgDataTx[2],
             pui8MsgDataTx[3]);
  
  // Send the CAN message using object number 1 (not the same thing as
  // CAN ID, which is also 1 in this example).  This function will cause
  // the message to be transmitted right away.
  //
  CANMessageSet(CAN0_BASE, MSG_OBJ_TX_NUM, &sCANMessageTx, MSG_OBJ_TYPE_TX);  
 
}

void HandleCANMsgRx(){
  
  uint8_t uIdx;
  
  // Reuse the same message object that was used earlier to configure
  // the CAN for receiving messages.  A buffer for storing the
  // received data must also be provided, so set the buffer pointer
  // within the message object.
  //
  sCANMessageRx.pui8MsgData = pui8MsgDataRx;
  // Read the message from the CAN.  Message object number 1 is used
  // (which is not the same thing as CAN ID).  The interrupt clearing
  // flag is not set because this interrupt was already cleared in
  // the interrupt handler.
  //
  CANMessageGet(CAN0_BASE, MSG_OBJ_RX_NUM, &sCANMessageRx, 0);
  //
  // Clear the pending message flag so that the interrupt handler can
  // set it again when the next message arrives.
  //
  g_bRXFlag = 0;
  
  //
  // Check to see if there is an indication that some messages were
  // lost.
  //
  if(sCANMessageRx.ui32Flags & MSG_OBJ_DATA_LOST)
  {
    UARTprintf("CAN message loss detected\n");
  }
  
  //
  // Print out the contents of the message that was received.
  //
  UARTprintf("Msg ID=0x%08X len=%u data=0x",
             sCANMessageRx.ui32MsgID, sCANMessageRx.ui32MsgLen);
  
  for(uIdx = 0; uIdx < sCANMessageRx.ui32MsgLen; uIdx++)
  {
    UARTprintf("%02X ", pui8MsgDataRx[uIdx]);
  }
  UARTprintf("total count=%u\n", g_ui32MsgRxCount);
}

