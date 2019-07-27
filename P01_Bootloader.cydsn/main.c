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

#define BUZZER_TIMER_CLK 100000
uint16 buzzer_freq = 20;
uint16 buzzer_clock_div = 2;
uint16 buzzer_timer_peorid = 1;
uint16 buzzer_timer_cmp1 = 0;

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    //PWM_BUZZER_Start();
    PWM_LED_Start();
    PWM_EN_Start();
    
    // Start Buzzer
    buzzer_freq = 4000;
    buzzer_clock_div = BUZZER_TIMER_CLK / buzzer_freq;
    buzzer_timer_peorid = buzzer_clock_div - 1;
    buzzer_timer_cmp1 = buzzer_clock_div/2-1;
    PWM_BUZZER_WritePeriod(buzzer_timer_peorid);
    PWM_BUZZER_WriteCompare(buzzer_timer_cmp1);
    //CyDelay(100);
    
    //CyDelay(5000);
        
    // Start bootloader
    Bootloader_Start();
    

    for(;;)
    {
        /* Place your application code here. */       

    }
}

/* [] END OF FILE */
