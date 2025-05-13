/*******************************************************************************
 * File Name        : main.c
 *
 * Description      : This source file contains the main routine for non-secure
 *                    application running on CM33 CPU.
 *
 * Related Document : See README.md
 *
 ********************************************************************************
 * Copyright 2024-2025, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
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
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

/*******************************************************************************
 * Header Files
 *******************************************************************************/

#include "cybsp.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cyabs_rtos.h"
#include "cyabs_rtos_impl.h"

/*******************************************************************************
 * Macros
 ******************************************************************************/
#define BLINKY_TASK_NAME            ("Blinky")
#define BLINKY_TASK_STACK_SIZE      (configMINIMAL_STACK_SIZE)
#define BLINKY_TASK_PRIORITY        (tskIDLE_PRIORITY + 1)
#define MAIN_TASK_NAME              ("Main")
#define MAIN_TASK_STACK_SIZE        (configMINIMAL_STACK_SIZE)
#define MAIN_TASK_PRIORITY          (tskIDLE_PRIORITY + 1)

/* USER LED toggle period in milliseconds */
#define USER_LED_TOGGLE_PERIOD_MS   1000u


/* Enabling or disabling a MCWDT requires a wait time of upto 2 CLK_LF cycles
 * to come into effect. This wait time value will depend on the actual CLK_LF
 * frequency set by the BSP.
 */
#define LPTIMER_0_WAIT_TIME_USEC            (62U)

/* Define the LPTimer interrupt priority number. '1' implies highest priority.
 */
#define APP_LPTIMER_INTERRUPT_PRIORITY      (1U)

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

/* LPTimer HAL object */
static mtb_hal_lptimer_t lptimer_obj;

/* RTOS semaphore */
static SemaphoreHandle_t xSemaphore;

/*******************************************************************************
 * Function Name: handle_app_error
 ********************************************************************************
 * Summary:
 * User defined error handling function
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void handle_app_error(void)
{
    /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);

    /* Infinite loop */
    while(true);
}

/*******************************************************************************
 * Function Name: blinky_task
 ********************************************************************************
 * Summary:
 *  This RTOS task toggles the User LED each time the semaphore is obtained.
 *
 * Parameters:
 *  void *pvParameters : Task parameter defined during task creation (unused)
 *
 * Return:
 *  The RTOS task never returns.
 *
 *******************************************************************************/

static void blinky_task(void *pvParameters)
{

    (void) pvParameters;

    for(;;)
    {
        /* Block until the semaphore is given */
        xSemaphoreTake(xSemaphore, portMAX_DELAY);

        /* Toggle the USER LED state */
        Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
    }
}

/*******************************************************************************
 * Function Name: main_task
 ********************************************************************************
 * Summary:
 *  This RTOS task releases the semaphore every USER_LED_TOGGLE_PERIOD_MS.
 *
 * Parameters:
 *  void *pvParameters : Task parameter defined during task creation (unused)
 *
 * Return:
 *  The RTOS task never returns.
 *
 *******************************************************************************/

static void main_task(void *pvParameters)
{
    (void) pvParameters;


    for(;;)
    {

        /* Block task for USER_LED_TOGGLE_PERIOD_MS. */
        vTaskDelay(USER_LED_TOGGLE_PERIOD_MS);

        /* Release semaphore */
        xSemaphoreGive(xSemaphore);
    }
}

/*******************************************************************************
 * Function Name: lptimer_interrupt_handler
 ********************************************************************************
 * Summary:
 * Interrupt handler function for LPTimer instance.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void lptimer_interrupt_handler(void)
{
    mtb_hal_lptimer_process_interrupt(&lptimer_obj);
}

/*******************************************************************************
 * Function Name: setup_tickless_idle_timer
 ********************************************************************************
 * Summary:
 * 1. This function first configures and initializes an interrupt for LPTimer.
 * 2. Then it initializes the LPTimer HAL object to be used in the RTOS
 *    tickless idle mode implementation to allow the device enter deep sleep
 *    when idle task runs. LPTIMER_0 instance is configured for CM33 CPU.
 * 3. It then passes the LPTimer object to abstraction RTOS library that
 *    implements tickless idle mode
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void setup_tickless_idle_timer(void)
{
    /* Interrupt configuration structure for LPTimer */
    cy_stc_sysint_t lptimer_intr_cfg =
    {
            .intrSrc = CYBSP_CM33_LPTIMER_0_IRQ,
            .intrPriority = APP_LPTIMER_INTERRUPT_PRIORITY
    };

    /* Initialize the LPTimer interrupt and specify the interrupt handler. */
    cy_en_sysint_status_t interrupt_init_status =
            Cy_SysInt_Init(&lptimer_intr_cfg,
                    lptimer_interrupt_handler);

    /* LPTimer interrupt initialization failed. Stop program execution. */
    if(CY_SYSINT_SUCCESS != interrupt_init_status)
    {
        handle_app_error();
    }

    /* Enable NVIC interrupt. */
    NVIC_EnableIRQ(lptimer_intr_cfg.intrSrc);

    /* Initialize the MCWDT block */
    cy_en_mcwdt_status_t mcwdt_init_status =
            Cy_MCWDT_Init(CYBSP_CM33_LPTIMER_0_HW,
                    &CYBSP_CM33_LPTIMER_0_config);

    /* MCWDT initialization failed. Stop program execution. */
    if(CY_MCWDT_SUCCESS != mcwdt_init_status)
    {
        handle_app_error();
    }

    /* Enable MCWDT instance */
    Cy_MCWDT_Enable(CYBSP_CM33_LPTIMER_0_HW,
            CY_MCWDT_CTR_Msk,
            LPTIMER_0_WAIT_TIME_USEC);

    /* Setup LPTimer using the HAL object and desired configuration as defined
     * in the device configurator. */
    cy_rslt_t result = mtb_hal_lptimer_setup(&lptimer_obj,
            &CYBSP_CM33_LPTIMER_0_hal_config);

    /* LPTimer setup failed. Stop program execution. */
    if(CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }

    /* Pass the LPTimer object to abstraction RTOS library that implements
     * tickless idle mode
     */
    cyabs_rtos_set_lptimer(&lptimer_obj);
}

/*******************************************************************************
 * Function Name: main
 *******************************************************************************
 * Summary:
 * This is the main function for CM33 non-secure application.
 *    1. It initializes the device and board peripherals.
 *    2. It sets up the LPTimer instance for CM33 CPU.
 *    4. It creates the FreeRTOS application task 'cm33_blinky_task'.
 *    5. It starts the RTOS task scheduler.
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
    BaseType_t retval;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board initialization failed. Stop program execution */
    if (CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }
    xSemaphore = xSemaphoreCreateBinary();
    if( xSemaphore == NULL )
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Setup the LPTimer instance for CM33 CPU. */
    setup_tickless_idle_timer();

    retval = xTaskCreate(blinky_task, BLINKY_TASK_NAME, BLINKY_TASK_STACK_SIZE, NULL, BLINKY_TASK_PRIORITY, NULL );
    if (retval != pdPASS)
    {
        CY_ASSERT(0);
    }

    retval = xTaskCreate(main_task, MAIN_TASK_NAME, MAIN_TASK_STACK_SIZE, NULL, MAIN_TASK_PRIORITY, NULL );
    if (retval != pdPASS)
    {
        CY_ASSERT(0);
    }

    /* Start the scheduler */
    vTaskStartScheduler();
}

/* [] END OF FILE */

