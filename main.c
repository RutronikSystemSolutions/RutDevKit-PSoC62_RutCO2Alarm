/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RutDevKit-RutCO2Alarm
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*  Created on: 2021-06-01
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Jonavos g. 30, Kaunas 44262, Lithuania
*  Author: GDR
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
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
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cy_usb_dev.h"
#include "cycfg_usbdev.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main_task.h"
#include "speech_task.h"
#include "hid_task.h"

#define USB_CONNECT_TIMEOUT		0

static void usb_high_isr(void);
static void usb_medium_isr(void);
static void usb_low_isr(void);
void handle_error(void);

/* USB Interrupt Configuration */
const cy_stc_sysint_t usb_high_interrupt_cfg =
{
    .intrSrc = (IRQn_Type) usb_interrupt_hi_IRQn,
    .intrPriority = 1U,
};
const cy_stc_sysint_t usb_medium_interrupt_cfg =
{
    .intrSrc = (IRQn_Type) usb_interrupt_med_IRQn,
    .intrPriority = 2U,
};
const cy_stc_sysint_t usb_low_interrupt_cfg =
{
    .intrSrc = (IRQn_Type) usb_interrupt_lo_IRQn,
    .intrPriority = 3U,
};

int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
    	handle_error();
    }

    __enable_irq();

    /*Initialize LEDs*/
    result = cyhal_gpio_init( LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    result = cyhal_gpio_init( LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    /* Initialize the USB device */
    Cy_USB_Dev_Init(USB_DEV_HW, &USB_DEV_config, &usb_drvContext, &usb_devices[0], &usb_devConfig, &usb_devContext);

    /* Initialize the HID Class */
    Cy_USB_Dev_HID_Init(&usb_hidConfig, &usb_hidContext, &usb_devContext);

    Cy_USB_Dev_RegisterEventsCallback(NULL , &usb_devContext);

    /* Initialize the USB interrupts */
    Cy_SysInt_Init(&usb_high_interrupt_cfg,   &usb_high_isr);
    Cy_SysInt_Init(&usb_medium_interrupt_cfg, &usb_medium_isr);
    Cy_SysInt_Init(&usb_low_interrupt_cfg,    &usb_low_isr);

    /* Enable the USB interrupts */
    NVIC_EnableIRQ(usb_high_interrupt_cfg.intrSrc);
    NVIC_EnableIRQ(usb_medium_interrupt_cfg.intrSrc);
    NVIC_EnableIRQ(usb_low_interrupt_cfg.intrSrc);

    /*Enable USB Driver*/
    Cy_USB_Dev_Connect(false, USB_CONNECT_TIMEOUT, &usb_devContext);

    /*Enable debug output via KitProg UART*/

    result = cy_retarget_io_init( KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }
    printf("\x1b[2J\x1b[;H");
    printf("CY8CKIT-062S2-43012 RutCO2Alarm Application.\r\n");


    speech_MsgQueue = xQueueCreate( 1, sizeof( speech_t ) );
    if( speech_MsgQueue == NULL )
    {
    	printf("Error: could not create a queue for speech_task.\r\n");
    	handle_error();
    }

    hid_MsgQueue = xQueueCreate( 3, sizeof( hid_msg_t ) );
    if( hid_MsgQueue == NULL )
    {
    	printf("Error: could not create a queue for speech_task.\r\n");
    	handle_error();
    }

    xTaskCreate(main_task, "main task", configMINIMAL_STACK_SIZE*8, NULL, configMAX_PRIORITIES - 3, &main_task_handle);
    if(main_task_handle == NULL)
    {
    	printf("Error: could not create main_task.\r\n");
    	handle_error();
    }

    xTaskCreate(speech_task, "speech task", configMINIMAL_STACK_SIZE*4, NULL, configMAX_PRIORITIES - 2, &speech_task_handle);
    if(speech_task_handle == NULL)
    {
    	printf("Error: could not create speech_task.\r\n");
    	handle_error();
    }

    xTaskCreate(hid_task, "hid task", configMINIMAL_STACK_SIZE*4, NULL, configMAX_PRIORITIES - 1, &hid_task_handle);
    if(hid_task_handle == NULL)
    {
    	printf("Error: could not create hid_task.\r\n");
    	handle_error();
    }

    vTaskStartScheduler();
    /* RTOS scheduler exited */
    /* Halt the CPU if scheduler exits */
    CY_ASSERT(0);
}

/***************************************************************************
* Function Name: usb_high_isr
********************************************************************************
* Summary:
*  This function process the high priority USB interrupts.
*
***************************************************************************/
static void usb_high_isr(void)
{
	__disable_irq();

    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(USB_DEV_HW, Cy_USBFS_Dev_Drv_GetInterruptCauseHi(USB_DEV_HW), &usb_drvContext);

    __enable_irq();
}


/***************************************************************************
* Function Name: usb_medium_isr
********************************************************************************
* Summary:
*  This function process the medium priority USB interrupts.
*
***************************************************************************/
static void usb_medium_isr(void)
{
    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(USB_DEV_HW, Cy_USBFS_Dev_Drv_GetInterruptCauseMed(USB_DEV_HW),
                               &usb_drvContext);
}


/***************************************************************************
* Function Name: usb_low_isr
********************************************************************************
* Summary:
*  This function process the low priority USB interrupts.
*
**************************************************************************/
static void usb_low_isr(void)
{
    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(USB_DEV_HW, Cy_USBFS_Dev_Drv_GetInterruptCauseLo(USB_DEV_HW),
                               &usb_drvContext);
}

void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

/* [] END OF FILE */
