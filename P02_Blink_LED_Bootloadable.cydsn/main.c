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
uint16 buzzer_freq = 20;
uint16 buzzer_clock_div = 2;
uint16 buzzer_timer_peorid = 1;
uint16 buzzer_timer_cmp1 = 0;

uint16 buzzer_chirp_dir = 1;

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    PWM_LED_Start();
    PWM_EN_Start();
    PWM_BUZZER_Start();
    

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */

    for(;;)
    {
        /* Place your application code here.
        //LED_R_Write(~KEY1_Read());
        //LED_G_Write(~KEY2_Read());
        */
        // Start Buzzer
        
        
        
        buzzer_freq = 400; //4000
        buzzer_clock_div = BUZZER_TIMER_CLK / buzzer_freq;
        buzzer_timer_peorid = buzzer_clock_div - 1;
        buzzer_timer_cmp1 = buzzer_clock_div/2-1;
        PWM_BUZZER_WritePeriod(buzzer_timer_peorid);
        PWM_BUZZER_WriteCompare(buzzer_timer_cmp1);

    }
}

/* [] END OF FILE */
