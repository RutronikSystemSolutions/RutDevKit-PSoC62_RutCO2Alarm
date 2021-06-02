/*
 * hid_task.h
 *
 *  Created on: 2021-04-30
 *      Author: GDR
 */

#ifndef HID_TASK_H_
#define HID_TASK_H_

#include "cy_usb_dev.h"
#include "cycfg_usbdev.h"

typedef struct hid_sensor
{
	 uint8_t HID_Buffer[4];
}hid_data_t;

typedef struct sensor_message
{
	hid_data_t sensor_data;
} hid_msg_t;

extern TaskHandle_t hid_task_handle;
extern QueueHandle_t hid_MsgQueue;
extern hid_data_t hid_data;
extern cy_stc_usbfs_dev_drv_context_t  usb_drvContext;
extern cy_stc_usb_dev_context_t        usb_devContext;
extern cy_stc_usb_dev_hid_context_t    usb_hidContext;

void hid_task(void *param);

#endif /* HID_TASK_H_ */
