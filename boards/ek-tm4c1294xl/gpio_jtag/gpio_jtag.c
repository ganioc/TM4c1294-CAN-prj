//*****************************************************************************
//
// gpio_jtag.c - Example to demonstrate recovering the JTAG interface.
//
// Copyright (c) 2013-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "drivers/buttons.h"

#include "drivers/yj_can.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>GPIO JTAG Recovery (gpio_jtag)</h1>
//!
//! This example demonstrates changing the JTAG pins into GPIOs, a with a
//! mechanism to revert them to JTAG pins.  When first run, the pins remain in
//! JTAG mode.  Pressing the USR_SW1 button will toggle the pins between JTAG
//! mode and GPIO mode.  Because there is no debouncing of the push button
//! (either in hardware or software), a button press will occasionally result
//! in more than one mode change.
//!
//! In this example, four pins (PC0, PC1, PC2, and PC3) are switched.
//!
//! UART0, connected to the ICDI virtual COM port and running at 115,200,
//! 8-N-1, is used to display messages from this application.
//
//*****************************************************************************

//*****************************************************************************
//
// System clock rate in Hz.
//
//*****************************************************************************
uint32_t g_ui32SysClock;
extern uint32_t g_ui32MsgCount;
extern bool g_bErrFlag;
extern bool g_bRXFlag;
//*****************************************************************************
//
// The current mode of pins PC0, PC1, PC2, and PC3.  When zero, the pins
// are in JTAG mode; when non-zero, the pins are in GPIO mode.
//
//*****************************************************************************
volatile uint32_t g_ui32Mode;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// The interrupt handler for the PB4 pin interrupt.  When triggered, this will
// toggle the JTAG pins between JTAG and GPIO mode.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
  uint8_t ui8Buttons;
  uint8_t ui8ButtonsChanged;
  
  //
  // Grab the current, debounced state of the buttons.
  //
  ui8Buttons = ButtonsPoll(&ui8ButtonsChanged, 0);
  
  //
  // If the USR_SW1 button has been pressed, and was previously not pressed,
  // start the process of changing the behavior of the JTAG pins.
  //
  if(BUTTON_PRESSED(USR_SW1, ui8Buttons, ui8ButtonsChanged))
  {
    //
    // Toggle the pin mode.
    //
    g_ui32Mode ^= 1;
    
    //
    // See if the pins should be in JTAG or GPIO mode.
    //
    if(g_ui32Mode == 0)
    {
      //
      // Change PC0-3 into hardware (i.e. JTAG) pins.
      //
      HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
      HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x01;
      HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) |= 0x01;
      HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
      HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x02;
      HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) |= 0x02;
      HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
      HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x04;
      HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) |= 0x04;
      HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
      HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x08;
      HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) |= 0x08;
      HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
      HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x00;
      HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = 0;
      
      //
      // Turn on the LED to indicate that the pins are in JTAG mode.
      //
      ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1,
                       GPIO_PIN_0);
    }
    else
    {
      //
      // Change PC0-3 into GPIO inputs.
      //
      HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
      HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x01;
      HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) &= 0xfe;
      HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
      HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x02;
      HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) &= 0xfd;
      HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
      HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x04;
      HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) &= 0xfb;
      HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
      HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x08;
      HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) &= 0xf7;
      HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
      HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x00;
      HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = 0;
      ROM_GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, (GPIO_PIN_0 | GPIO_PIN_1 |
                                                 GPIO_PIN_2 |
                                                   GPIO_PIN_3));
      
      //
      // Turn off the LED to indicate that the pins are in GPIO mode.
      //
      ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1,
                       GPIO_PIN_1);
    }
  }
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
  //
  // Enable the GPIO Peripheral used by the UART.
  //
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  
  //
  // Enable UART2
  //
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
  
  //
  // Configure GPIO Pins for UART mode.
  //
  ROM_GPIOPinConfigure(GPIO_PD4_U2RX);
  ROM_GPIOPinConfigure(GPIO_PD5_U2TX);
  ROM_GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);
  
  //
  // Initialize the UART for console I/O.
  //
  UARTStdioConfig(2, 115200, g_ui32SysClock);
}

//*****************************************************************************
//
// Toggle the JTAG pins between JTAG and GPIO mode with a push button selecting
// between the two.
//
//*****************************************************************************
int
main(void)
{
  //uint32_t ui32Mode;
  uint8_t data[8]={0xff , 0xee, 0xdd, 0xcc, 0xbb, 0xaa, 0x00, 0x99};
  //
  // Set the clocking to run directly from the crystal at 120MHz.
  //
  g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                           SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                               SYSCTL_CFG_VCO_480), 120000000);
  //
  // Enable the peripherals used by this application.
  //
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  
  
  //
  // Initialize the button driver.
  //
  //ButtonsInit();
  
  //
  // Set up a SysTick Interrupt to handle polling and debouncing for our
  // buttons.
  //
  //SysTickPeriodSet(g_ui32SysClock / 100);
  //SysTickIntEnable();
  //SysTickEnable();
  
  // Init CAN0 
  InitCAN(500000);
  
  
  IntMasterEnable();
  
  //
  // Configure the LEDs as outputs and turn them on in the JTAG state.
  //
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_0);
  
  //
  // Initialize the UART, clear the terminal, print banner.
  //
  ConfigureUART();
  UARTprintf("\033[2J\033[H");
  UARTprintf("GPIO <-> JTAG\n");
  
  //
  // Indicate that the pins start out as JTAG.
  //
  UARTprintf("%d\n", g_ui32SysClock);
  
  InitCANTxMsg(0x0201, data, 3);
  InitCANRxMsg(0x0202);
  //
  // Loop forever.  This loop simply exists to display on the UART the
  // current state of PC0-3; the handling of changing the JTAG pins to and
  // from GPIO mode is done in GPIO Interrupt Handler.
  
  ReceiveCANMsg();
    
  //
  while(1)
  {
    
    //
    // Now wait 1 second before continuing
    //
    //UARTprintf("-->\n");
    //SimpleDelay();
    
    //SendCANMsg();
    
    if(g_bErrFlag)
    {
      UARTprintf(" error - cable connected?\n");
    }
    
    if(g_bRXFlag)
    {
      UARTprintf(" Message received\n");
      HandleCANMsgRx();
    }
    
  }
}
