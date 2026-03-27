/******************************************************************************
 * File Name: main.c
 *
 * Description: This is the source code for the PSoC 4 MCU TCPWM Interrupt 
 *              example for ModusToolbox.
 *
 * Related Document: See README.md
 *
 *
 ******************************************************************************
 * (c) 2020-2026, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
 *****************************************************************************/

/******************************************************************************
 * Include header files
 *****************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/* Timer period in milliseconds */
#define TIMER_PERIOD_MSEC   1000U

/*******************************************************************************
* Function Name: Timer_Interrupt_Handler
********************************************************************************
* Summary: 
* Handler function for the timer interrupt that simply toggles the LED.
*
* Parameters:
*  None
*
* Return:
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
* Summary:
* The main function performs the following actions:
*    1. Initializes the BSP
*    2. Calls the function to set up the TCPWM Counter
*    3. Calls the function to set up the interrupt on terminal count
*
* Parameters:
*  void
*
* Return:
*  int
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
        /*.intrSrc =*/ CYBSP_TIMER_IRQ,    /* Interrupt source is Timer interrupt */
        /*.intrPriority =*/ 3UL            /* Interrupt priority is 3 */
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

    /* Initialize the TCPWM component in timer/counter mode. The return value of the
     * function indicates whether the arguments are valid or not.
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
