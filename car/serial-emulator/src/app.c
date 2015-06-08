/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "xc.h"

void turnLeft(void);
void turnRight(void);
void straight(void);



// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;
uint8_t message[256] __attribute__((coherent, aligned(4)));


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************
 * USB CDC Device Events - Application Event Handler
 *******************************************************/

USB_DEVICE_CDC_EVENT_RESPONSE APP_USBDeviceCDCEventHandler
(
    USB_DEVICE_CDC_INDEX index ,
    USB_DEVICE_CDC_EVENT event ,
    void* pData,
    uintptr_t userData
)
{
    APP_DATA * appDataObject;
    appDataObject = (APP_DATA *)userData;
    USB_CDC_CONTROL_LINE_STATE * controlLineStateData;
    uint16_t * breakData;

    switch ( event )
    {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

            /* This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.  */

             USB_DEVICE_ControlSend(appDataObject->deviceHandle,
                    &appDataObject->getLineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

            /* This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host */

            USB_DEVICE_ControlReceive(appDataObject->deviceHandle,
                    &appDataObject->setLineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

            /* This means the host is setting the control line state.
             * Read the control line state. We will accept this request
             * for now. */

            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *)pData;
            appDataObject->controlLineStateData.dtr = controlLineStateData->dtr;
            appDataObject->controlLineStateData.carrier =
                    controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:

            /* This means that the host is requesting that a break of the
             * specified duration be sent. Read the break duration */

            breakData = (uint16_t *)pData;
            appDataObject->breakData = *breakData;
            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:

            /* This means that the host has sent some data*/
            appDataObject->isReadComplete = true;
            appDataObject->readLength =
                    ((USB_DEVICE_CDC_EVENT_DATA_READ_COMPLETE*)pData)->length;
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* The data stage of the last control transfer is
             * complete. We have received the line coding from the host. Call
             * the usart driver routine to change the baud. We can call this
             * function here as it is not blocking. */

            DRV_USART_BaudSet(appDataObject->usartHandle, appDataObject->setLineCodingData.dwDTERate);

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This means the GET LINE CODING function data is valid. We dont
             * do much with this data in this demo. */
            break;

        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the data write got completed. We can schedule
             * the next read. */

            break;

        default:
            break;
    }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

/*******************************************************
 * USB Device Layer Events - Application Event Handler
 *******************************************************/

void APP_USBDeviceEventHandler ( USB_DEVICE_EVENT event, void * eventData, uintptr_t context)
{
    uint8_t configurationValue;
    switch ( event )
    {
        case USB_DEVICE_EVENT_RESET:
        case USB_DEVICE_EVENT_DECONFIGURED:

            /* USB device is reset or device is deconfigured.
             * This means that USB device layer is about to deininitialize
             * all function drivers. Update LEDs to indicate
             * reset/deconfigured state. */

            BSP_LEDOn ( APP_USB_LED_1 );
            BSP_LEDOn ( APP_USB_LED_2  );
            BSP_LEDOff( APP_USB_LED_3  );

            appData.isConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuration */
            configurationValue = ((USB_DEVICE_EVENT_DATA_CONFIGURED *)eventData)->configurationValue;
            if (configurationValue == 1)
            {
                /* The device is in configured state. Update LED indication */

                BSP_LEDOff( APP_USB_LED_1 );
                BSP_LEDOff( APP_USB_LED_2  );
                BSP_LEDOn( APP_USB_LED_3  );

                /* Register the CDC Device application event handler here.
                 * Note how the appData object pointer is passed as the
                 * user data */

                USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0,
                        APP_USBDeviceCDCEventHandler, (uintptr_t)&appData);

                /* mark that set configuration is complete */
                appData.isConfigured = true;

            }
            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            /* Update LEDs */
            BSP_LEDOff ( APP_USB_LED_1 );
            BSP_LEDOn ( APP_USB_LED_2  );
            BSP_LEDOn( APP_USB_LED_3  );
            break;

        
        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. Connect the device */
            USB_DEVICE_Attach (appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:
            
            /* VBUS was removed. Disconnect the device */
            USB_DEVICE_Detach(appData.deviceHandle);
            break;

        /* These events are not used in this demo */
        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/*****************************************************
 * This function is called in every step of the
 * application state machine.
 *****************************************************/

bool APP_StateReset(void)
{
    /* This function returns true if the device
     * was reset  */

    bool retVal;

    if(appData.isConfigured == false)
    {
        appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
        appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.isReadComplete = true;
        appData.isWriteComplete = true;

        retVal = true;
    }
    else
    {
        retVal = false;
    }

    return(retVal);
}



// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    ANSELBbits.ANSB14 = 0; // 0 for digital, 1 for analog
    ANSELBbits.ANSB15 = 0; // 0 for digital, 1 for analog
//For an input function, use the (funtion name)Rbits.(function name) to set the pin, using table 12-1 in Chapter 12
//For an output function, use the RP(pin name)Rbits.RP(pin name)R to set the pin, using table 12-2 in Chapter 12
    // set up USER pin as input
    TRISBbits.TRISB14 = 0; // make pin 14 an output
    TRISBbits.TRISB7 = 0; // make pin 7 an output
    RPB15Rbits.RPB15R = 0b0101; // set B15 to OC1
    T2CONbits.TCKPS = 2; // Timer2 prescaler N=4 (1:4)
    PR2 = 19999; // period = (PR2+1) * N * 12.5 ns = 1000 us, 1 kHz
    TMR2 = 0; // initial TMR2 count is 0
    OC1RS = 0;
    OC1CONbits.OCTSEL = 0; // use Timer2 for OC1
    OC1CONbits.OCM = 0b110; // PWM mode with fault pin disabled
    T2CONbits.ON = 1; // turn on Timer2
    OC1CONbits.ON = 1; // turn on OC1
    //setup Pin5 as OC1 using timer3 at 1kHz
    RPB5Rbits.RPB5R = 0b0101; //B5 to OC2
    T3CONbits.TCKPS = 2;
    PR3 = 19999;
    OC2RS = 0;
    OC2CONbits.OCTSEL = 0; // use Timer3 for OC2
    OC2CONbits.OCM = 0b110; // PWM mode with fault pin disabled
    T3CONbits.ON = 1; // turn on Timer3
    OC2CONbits.ON = 1; //turn on OC2
    LATBbits.LATB14 = 0; //pin 14 sends low

    LATBbits.LATB7  = 1; //pin 7 sends low
     /* Device Layer Handle  */
    appData.deviceHandle = USB_DEVICE_HANDLE_INVALID;

    /* USART Driver Handle */
    appData.usartHandle = DRV_HANDLE_INVALID;

    /* CDC Instance index for this app object00*/
    appData.cdcInstance = USB_DEVICE_CDC_INDEX_0;

    /* app state */
    appData.state = APP_STATE_INIT ;

    /* device configured status */
    appData.isConfigured = false;

    /* Initial get line coding state */
    appData.getLineCodingData.dwDTERate = 9600;
    appData.getLineCodingData.bDataBits = 8;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bCharFormat = 0;

    /* Read Transfer Handle */
    appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Write Transfer Handle */
    appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Intialize the read complete flag */
    appData.isReadComplete = true;

    /*Initialize the write complete flag*/
    appData.isWriteComplete = true;

    turnRight();
    turnLeft();
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
USB_DEVICE_CDC_RESULT writeRequestResult;


    /* Update the application state machine based
     * on the current state */

    switch(appData.state)
    {
        case APP_STATE_INIT:

            /* Open the device layer */
            appData.deviceHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0,    DRV_IO_INTENT_READWRITE );

            if(appData.deviceHandle != USB_DEVICE_HANDLE_INVALID)
            {
                appData.usartHandle = DRV_USART_Open(DRV_USART_INDEX_0,
                    (DRV_IO_INTENT_EXCLUSIVE|DRV_IO_INTENT_READWRITE|DRV_IO_INTENT_NONBLOCKING));

                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.deviceHandle, APP_USBDeviceEventHandler, 0);

                

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            }
            else
            {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device was configured */
            if(appData.isConfigured)
            {
                /* Schedule the first read from CDC function driver */

                appData.state = APP_STATE_CHECK_CDC_READ;
                appData.isReadComplete = false;
                USB_DEVICE_CDC_Read (appData.cdcInstance, &(appData.readTransferHandle),
                        appData.readBuffer, 64);
            }
            break;

        case APP_STATE_CHECK_CDC_READ:

            if(APP_StateReset())
            {
                break;
            }

            /* If a read is complete, then schedule a read
             * else check if UART has received data */

            if(appData.isReadComplete == true)
            {
                appData.isReadComplete = false;
                USB_DEVICE_CDC_Read (appData.cdcInstance, &appData.readTransferHandle,
                        appData.readBuffer, 64);

                int COM;

                sscanf(appData.readBuffer, "%d", &COM);

                

                //compare curent and previous values if different within tolerance
                //turn left or right
                if(COM < 320-2*640/5){
                    turnLeft();
                }
                if(COM > 320+2*640/5 ){
                    turnRight();
                }
                else{
                    straight();
                }



            }


                //DRV_USART_Write(appData.usartHandle, appData.readBuffer, appData.readLength);

            appData.state = APP_STATE_CHECK_UART_RECEIVE;
            break;

        case APP_STATE_CHECK_UART_RECEIVE:

            if(APP_StateReset())
            {
                break;
            }

            /* Check if a character was received on the UART */

//            if(DRV_USART_Read(appData.usartHandle, &appData.uartReceivedData, 1) > 0)
//            {
//                /* We have received data on the UART */
//
//                USB_DEVICE_CDC_Write(0, &appData.writeTransferHandle,
//                        &appData.uartReceivedData, 1,
//                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
//
//            }
          int len = 0;
          while(message[len]>0){
            len++;
          }
          writeRequestResult = USB_DEVICE_CDC_Write(0, &appData.writeTransferHandle, message, len, USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);

          if(USB_DEVICE_CDC_RESULT_OK != writeRequestResult) {
          }
          else {
          }

            appData.state = APP_STATE_CHECK_CDC_READ;
            break;

        case APP_STATE_ERROR:
            break;
        default:
            break;
    }
}

void turnLeft ( void ) {
//    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT()<10000000) {
        OC1RS = 15000;
        OC2RS = 15000;
    }
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT()<4000000) {
        OC1RS = 15000;
        OC2RS = 5000;
    }
    
}

void turnRight ( void ) {
    //OC1 is right wheel
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT()<10000000) {
        OC1RS = 15000;
        OC2RS = 15000;
    }
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT()<8000000) {
        OC1RS = 5000;
        OC2RS = 15000;
    }
    

}
void straight ( void ) {
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT()<10000000) {
        OC1RS = 15000;
        OC2RS = 15000;
    }
}


/*******************************************************************************
 End of File
 */

