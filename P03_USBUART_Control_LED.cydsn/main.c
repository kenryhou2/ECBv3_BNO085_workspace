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
#include "project.h"

#define BUZZER_TIMER_CLK 8000000

/// USBUART
#define USBFS_DEVICE    (0u)
#define USBUART_BUFFER_SIZE (64u)
uint16 count;
uint8 buffer[USBUART_BUFFER_SIZE];

#define SERIAL_LED_R_MASK 0x4
#define SERIAL_LED_G_MASK 0x2
#define SERIAL_LED_B_MASK 0x1


uint16 buzzer_freq = 20;
uint16 buzzer_clock_div = 2;
uint16 buzzer_timer_peorid = 1;
uint16 buzzer_timer_cmp1 = 0;

uint16 buzzer_chirp_dir = 1;

uint8 serial_input = 0;

void USBUART_user_check_init(void);
void USBUART_user_echo(void);



int main(void)
{
   
    CyGlobalIntEnable; /* Enable global interrupts. */
    USBUART_Start(USBFS_DEVICE, USBUART_5V_OPERATION);
    
    while(0 == USBUART_GetConfiguration())  //wait for enumeration
    {
        
    } 
    USBUART_CDC_Init();
    LED_R_Write(0);
    CyDelay(3000);
    USBUART_PutString("Start\r\n");
    USBUART_PutString("Type any letter\r\n");
    
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */

    for(;;)
    {
        /* Place your application code here. 
        //LED_R_Write(~KEY1_Read());
        //LED_G_Write(~KEY2_Read());
        */
        
        // Start Buzzer
        //buzzer_freq = 4000;
        //buzzer_clock_div = BUZZER_TIMER_CLK / buzzer_freq;
        //buzzer_timer_peorid = buzzer_clock_div - 1;
        //buzzer_timer_cmp1 = buzzer_clock_div/2-1;
        //PWM_BUZZER_WritePeriod(buzzer_timer_peorid);
        //PWM_BUZZER_WriteCompare(buzzer_timer_cmp1);
        
        USBUART_user_check_init();
        USBUART_user_echo();
        while (0u == USBUART_GetConfiguration());
        
        serial_input = USBUART_GetChar();
        serial_input = buffer[0];
        
        LED_R_Write((serial_input & SERIAL_LED_R_MASK) >> 2);
        LED_G_Write((serial_input & SERIAL_LED_G_MASK) >> 1);
        LED_B_Write((serial_input & SERIAL_LED_B_MASK) >> 0);
        
        CyDelay(10);
        

    }
}


/// USBUART Routin
void USBUART_user_check_init(void) {
    /* Host can send double SET_INTERFACE request. */
    if (0u != USBUART_IsConfigurationChanged())
    {
        /* Initialize IN endpoints when device is configured. */
        if (0u != USBUART_GetConfiguration())
        {
            /* Enumeration is done, enable OUT endpoint to receive data 
             * from host. */
            USBUART_CDC_Init();
        }
    }
}


void USBUART_user_echo(void) {
    /* Service USB CDC when device is configured. */
    if (0u != USBUART_GetConfiguration())
    {
        /* Check for input data from host. */
        if (0u != USBUART_DataIsReady())
        {
            /* Read received data and re-enable OUT endpoint. */
            count = USBUART_GetAll(buffer);

            if (0u != count)
            {
                /* Wait until component is ready to send data to host. */
                while (0u == USBUART_CDCIsReady())
                {
                }

                /* Send data back to host. */
                USBUART_PutData(buffer, count);
                USBUART_PutString("Data Sent\n");
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
    }
}

/* [] END OF FILE */
