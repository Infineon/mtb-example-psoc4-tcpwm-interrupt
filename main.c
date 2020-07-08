/******************************************************************************
* File Name:  main.c
*
* Description:  This is the source code for the PSoC 4 MCU TCPWM Interrupt 
*               example
*
* Related Document: See Readme.md
*
*******************************************************************************
* (c) 2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/



/* Header file includes */
#include "cy_pdl.h"
#include "cybsp.h"

/* Timer period in milliseconds */
#define TIMER_PERIOD_MSEC   1000U

/*******************************************************************************
 * Function Name: Timer_Interrupt_Handler
 ********************************************************************************
 *
 * Summary: Handler function for the timer interrupt that simply toggles the LED.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
 * Side Effects:
 *  None
 *
 *******************************************************************************/

void timer_interrupt_handler(void)
{
    /* Clear the terminal count interrupt */
    Cy_TCPWM_ClearInterrupt(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, CY_TCPWM_INT_ON_TC );

    /* Toggle the LED */
    Cy_GPIO_Inv(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_NUM);
}


/*******************************************************************************
 * Function Name: main
 *******************************************************************************
 *
 *  The main function performs the following actions:
 *   1. Initializes the BSP
 *   2. Calls the function to set up the TCPWM Counter
 *   3. Calls the function to set up the interrupt on Terminal Count
 *
 ******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the interrupt vector table with the timer interrupt handler
     * address and assign priority.
     */
    cy_stc_sysint_t intrCfg =
    {
       /*.intrSrc =*/ CYBSP_TIMER_IRQ, /* Interrupt source is Timer interrupt */
       /*.intrPriority =*/ 3UL   /* Interrupt priority is 3 */
    };

    result = Cy_SysInt_Init(&intrCfg, timer_interrupt_handler);
    if(result != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable Interrupt */
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* Enable global interrupts */
    __enable_irq();

    /* Start the TCPWM component in timer/counter mode. The return value of the
     * function indicates whether the arguments are valid or not. It is not used
     * here for simplicity.
     */
    result = Cy_TCPWM_Counter_Init(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, &CYBSP_TIMER_config);
    if(result != CY_TCPWM_SUCCESS)
    {
        CY_ASSERT(0);
    }

    Cy_TCPWM_Counter_Enable(CYBSP_TIMER_HW, CYBSP_TIMER_NUM);

    /* Check if the desired interrupt is enabled prior to triggering */
    if (0UL != (CY_TCPWM_INT_ON_TC & Cy_TCPWM_GetInterruptMask(CYBSP_TIMER_HW, CYBSP_TIMER_NUM)))
    {
       Cy_TCPWM_SetInterrupt(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, CY_TCPWM_INT_ON_TC);
    }

    /* Set the timer period in milliseconds. To count N cycles, period should be
     * set to N-1.
     */
    Cy_TCPWM_Counter_SetPeriod(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, TIMER_PERIOD_MSEC-1 );

    /* Trigger a software start on the counter instance. This is required when
     * no other hardware input signal is connected to the component to act as
     * a trigger source.
     */
    Cy_TCPWM_TriggerStart(CYBSP_TIMER_HW, CYBSP_TIMER_MASK);

    for (;;)
    {
    }
}

/* [] END OF FILE */
