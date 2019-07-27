/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

// TODOs
// PD output change 5V
// CAN led change dir
// BNO085 getting hot


#include "project.h"

/// USBUART
#define USBFS_DEVICE    (0u)
#define USBUART_BUFFER_SIZE (64u)
uint16 count;
uint8 buffer[USBUART_BUFFER_SIZE];
void USBUART_user_send(void);

//CAN
uint8 Rx_Data = 0u;


int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
	
	// Start USBEART
	USBUART_Start(USBFS_DEVICE, USBUART_5V_OPERATION);
    USBUART_CDC_Init();
	
	/* Initialize and Start CAN component */
    CAN_RX_Init();
    CAN_RX_Start(); 
	

    for(;;)
    {
           
		LED_G_Write(~LED_G_Read());
		CyDelay(1000);

    }
}

void USBUART_user_send(void) {
    /* Service USB CDC when device is configured. */
    if (0u != USBUART_GetConfiguration())
    {
        /* Wait until component is ready to send data to host. */
        while (0u == USBUART_CDCIsReady())
        {
        }

        /* Send data back to host. */
		buffer[0] = 0x31;
	    buffer[1] = 0x30 + Rx_Data;
		//buffer[1] = 0x32;
		buffer[2] = 0x33;
		buffer[3] = 0x0D; // \r
		buffer[4] = 0x0A; // \n
		count = 5;
        USBUART_PutData(buffer, count);
		
		//USBUART_PutChar(0xAA);        /*UART frame header*/   
		//USBUART_PutChar(Tx_Data);	/*UART data*/
		//USBUART_PutChar(0x55);        /*UART frame tail*/

        /* If the last sent packet is exactly the maximum packet 
        *  size, it is followed by a zero-length packet to assure
        *  that the end of the segment is properly identified by 
        *  the terminal.
        */
        if (USBUART_BUFFER_SIZE == count)
        {
            /* Wait until component is ready to send data to PC. */
            while (0u == USBUART_CDCIsReady())
            {
            }

            /* Send zero-length packet to PC. */
            USBUART_PutData(NULL, 0u);
        }
    }
}

/* [] END OF FILE */
