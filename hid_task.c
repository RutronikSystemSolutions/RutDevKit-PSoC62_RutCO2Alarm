/*
 * hid_task.c
 *
 *  Created on: 2021-04-30
 *      Author: GDR
 */

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cy_usb_dev.h"
#include "cycfg_usbdev.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "hid_task.h"

#define HID_ENDPOINT     		0x01
#define MAX_NUM_BYTES       	0x04

TaskHandle_t hid_task_handle = NULL;
QueueHandle_t hid_MsgQueue;
hid_msg_t hid_msg;

hid_data_t hid_data =
{
		.HID_Buffer[0] = 0x00,
		.HID_Buffer[1] = 0x00,
		.HID_Buffer[2] = 0x00,
		.HID_Buffer[3] = 0x00,
};

/* USBDEV context variables */
cy_stc_usbfs_dev_drv_context_t  usb_drvContext;
cy_stc_usb_dev_context_t        usb_devContext;
cy_stc_usb_dev_hid_context_t    usb_hidContext;

void hid_task(void *param)
{
	(void) param;
	hid_msg_t msg = {0};
	BaseType_t msg_status;
	cy_en_usb_dev_ep_state_t epState;

	printf("hid_task has started.\r\n");

	for(;;)
	{
		msg_status = xQueueReceive( hid_MsgQueue, &(msg), portMAX_DELAY);
		if (msg_status == pdPASS)
		{
			if(usb_devContext.state == CY_USB_DEV_CONFIGURED)
			{
				epState = Cy_USBFS_Dev_Drv_GetEndpointState(USB_DEV_HW, HID_ENDPOINT, &usb_drvContext);
				if ((CY_USB_DEV_EP_IDLE == epState) || (CY_USB_DEV_EP_COMPLETED == epState))
				{
			        /* Send out report data */
			        Cy_USB_Dev_WriteEpNonBlocking(HID_ENDPOINT, msg.sensor_data.HID_Buffer, MAX_NUM_BYTES, &usb_devContext);
			        cyhal_gpio_toggle(LED1);
				}
			}
		}
	}
}
