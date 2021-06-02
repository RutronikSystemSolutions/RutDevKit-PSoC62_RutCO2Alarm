/*
 * main_task.c
 *
 *  Created on: 2021-04-29
 *      Author: GDR
 */

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "spi_api.h"
#include "sensirion_common.h"
#include "sgp30.h"
#include "main_task.h"
#include "speech_task.h"
#include "hid_task.h"

TaskHandle_t main_task_handle = NULL;
TimerHandle_t phrase_repeat = NULL;
TimerHandle_t speech_check = NULL;
long ph_rep_id = 1;
long sp_ch_id = 2;
co2status_t co2_state_prev = CO2_UNDEFINED;
int co2_buff[10] = {0};
int buff_pos = 0;
long sum = 0;
_Bool check = true;

static void Speech_Check_Callback(TimerHandle_t xTimer);
static void Phrase_Repeat_Callback(TimerHandle_t xTimer);
static co2status_t DecodeCO2Level(int co2eq_ppm);
static int MovingAvg(int *ptrArrNumbers, long *ptrSum, int pos, int len, int nextNum);

void main_task(void *param)
{
	(void) param;
	cy_rslt_t result;
	int ecfg_result = -1;
	BaseType_t msg_status;
	speech_t speech_msg;
	uint16_t tvoc_ppb = 0;
	uint16_t co2_eq_ppm = 0;
	co2status_t co2_state = CO2_UNDEFINED;
	int co2_av = 0;
	int buff_len;
	hid_msg_t hid_msg;

#ifdef DEBUG_CO2_VALUE
	uint8_t cnt = 0;
#endif

	printf("main task has started.\r\n");

	/*Initialize Epson ASIC*/
	result = EPSON_Initialize();
	if(result != CY_RSLT_SUCCESS)
	{
		printf("Text-to-Speech Engine failed to initialize.\r\n");
		 CY_ASSERT(0);
	}

	/*Epson ASIC Hard-Reset Procedure*/
	GPIO_S1V30340_Reset(1);
	vTaskDelay(pdMS_TO_TICKS(100));
	GPIO_S1V30340_Reset(0);
	vTaskDelay(pdMS_TO_TICKS(100));
	GPIO_S1V30340_Reset(1);
	vTaskDelay(pdMS_TO_TICKS(500));

	/*Set Epson ASIC mute signal(MUTE) to High(disable)*/
	GPIO_ControlMute(0);

	/*Set Epson ASIC standby signal(STBYEXIT) to Low(deassert)*/
	GPIO_ControlStandby(0);
	vTaskDelay(pdMS_TO_TICKS(100));

	/*Configure Epson ASIC*/
	ecfg_result = S1V30340_Initialize_Audio_Config();
	while(ecfg_result != 0)
	{
		printf("S1V30340 SPI Initialization failed. \n\r");
		vTaskDelay(pdMS_TO_TICKS(1000));
		ecfg_result = S1V30340_Initialize_Audio_Config();
	}
	printf("S1V30340 SPI Initialization succeeded. \n\r");

	/*Play the greeting*/
	GPIO_ControlMute(1); /*Mute - OFF*/
	S1V30340_Play_Specific_Audio(GREETING_PHRASE);
	S1V30340_Wait_For_Termination();
	GPIO_ControlMute(0); /*Mute - ON*/

	/*Initialize SGP30 sensor*/
	while(sgp_probe() != STATUS_OK)
	{
		printf("SGP sensor probing failed ... check SGP30 I2C connection and power \n\r");
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	printf("Sensor probing successful \r\n");
	sgp_iaq_init();

	for(;;)
	{
		/*Task executed 10 times per second*/
		vTaskDelay(pdMS_TO_TICKS(100));

		/*Do the measurement*/
	    if(!(sgp_measure_iaq_blocking_read(&tvoc_ppb, &co2_eq_ppm) == STATUS_OK))
	    {
	    	co2_eq_ppm = 0;
	    	tvoc_ppb = 0;
	    	printf("Could not read CO2 sensor values \r\n");
	    	vTaskDelay(pdMS_TO_TICKS(1000));
	    }

#ifdef DEBUG_CO2_VALUE
	    if(cnt == 9)
	    {
	    	cnt = 0;
	    	printf("CO2 equivalent value: %d ppm\r\n", co2_eq_ppm);
	    }
	    cnt++;
#endif

	    /*Do the moving average math*/
	    buff_len = sizeof(co2_buff)/sizeof(int);
	    co2_av = MovingAvg(co2_buff, &sum, buff_pos, buff_len, (int)co2_eq_ppm);
	    buff_pos++;
	    if(buff_pos >= buff_len){buff_pos = 0;}

	    /*Wrap averaged sensor value into the message*/
	    hid_msg.sensor_data.HID_Buffer[0] = (co2_av >> 24);
	    hid_msg.sensor_data.HID_Buffer[1] = (co2_av >> 16);
	    hid_msg.sensor_data.HID_Buffer[2] = (co2_av >> 8);
	    hid_msg.sensor_data.HID_Buffer[3] = (co2_av >> 0);

		msg_status = xQueueSend(hid_MsgQueue, (void *)&(hid_msg), pdMS_TO_TICKS(1000) );
		if(msg_status != pdPASS)
		{
			printf("Timeout: Could not send the HID message.\n\r");
		}

		/*Speech engine periodic check*/
		if(check)
		{
			/*Self lock-out*/
			check = false;

			/*Check the current status of the CO2 concentration*/
			co2_state = DecodeCO2Level(co2_av);

			/*Play warning messages according to the CO2 concentration*/
			msg_status = pdFAIL;
			switch (co2_state)
			{
				case CO2_NORMAL:
				{
					/*Play the message only once on state change*/
					if(co2_state_prev != CO2_NORMAL)
					{
						speech_msg.phrase_number = CO2_NORMAL_PHRASE;
						msg_status = xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 0 );
						if(msg_status == pdPASS)
						{
							if(phrase_repeat != NULL)
							{
								xTimerDelete(phrase_repeat, 100);
								phrase_repeat = NULL;
							}

							co2_state_prev = CO2_NORMAL;
						}
					}
					break;
				}
				case CO2_MINIMUM:
				{
					/*Play the message only once on state change*/
					if(co2_state_prev != CO2_MINIMUM)
					{
						speech_msg.phrase_number = CO2_MINIMUM_PHRASE;
						msg_status = xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 0 );
						if(msg_status == pdPASS)
						{
							if(phrase_repeat != NULL)
							{
								xTimerDelete(phrase_repeat, 100);
								phrase_repeat = NULL;
							}

							co2_state_prev = CO2_MINIMUM;
						}
					}
					break;
				}
				case CO2_MEDIUM:
				{
					/*Play the message every xx seconds*/
					if(co2_state_prev != CO2_MEDIUM)
					{
						speech_msg.phrase_number = CO2_MEDIUM_PHRASE;
						msg_status = xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 0 );
						if(msg_status == pdPASS)
						{
							if(phrase_repeat != NULL)
							{
								xTimerDelete(phrase_repeat, 100);
								phrase_repeat = NULL;
							}

							co2_state_prev = CO2_MEDIUM;

							phrase_repeat = xTimerCreate("Repeat", CO2_MEDIUM_REPEAT, pdFALSE, &ph_rep_id, Phrase_Repeat_Callback);
							xTimerStart(phrase_repeat, 100);
						}
					}
					break;
				}
				case CO2_MAXIMUM:
				{
					/*Play the message every xx seconds*/
					if(co2_state_prev != CO2_MAXIMUM)
					{
						speech_msg.phrase_number = CO2_MAXIMUM_PHRASE;
						msg_status = xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 0 );
						if(msg_status == pdPASS)
						{
							if(phrase_repeat != NULL)
							{
								xTimerDelete(phrase_repeat, 100);
								phrase_repeat = NULL;
							}

							co2_state_prev = CO2_MAXIMUM;

							phrase_repeat = xTimerCreate("Repeat", CO2_MAXIMUM_REPEAT, pdFALSE, &ph_rep_id, Phrase_Repeat_Callback);
							xTimerStart(phrase_repeat, 100);
						}
					}
					break;
				}
				case CO2_EXTREME:
				{
					/*Play the message every xx seconds*/
					if(co2_state_prev != CO2_EXTREME)
					{
						speech_msg.phrase_number = CO2_EXTREME_PHRASE;
						msg_status = xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 0 );
						if(msg_status == pdPASS)
						{
							if(phrase_repeat != NULL)
							{
								xTimerDelete(phrase_repeat, 100);
								phrase_repeat = NULL;
							}

							co2_state_prev = CO2_EXTREME;

							phrase_repeat = xTimerCreate("Repeat", CO2_EXTREME_REPEAT, pdFALSE, &ph_rep_id, Phrase_Repeat_Callback);
							xTimerStart(phrase_repeat, 100);
						}
					}
					break;
				}
				case CO2_UNDEFINED:
				{
					break;
				}
				default:
				{
					break;
				}
			}

			/*Start the timer which will activate next speech engine check*/
			speech_check = xTimerCreate("Check", SPEECH_ENGINE_CHECK, pdFALSE, &sp_ch_id, Speech_Check_Callback);
			xTimerStart(speech_check, 100);
		}
	}
}

static co2status_t DecodeCO2Level(int co2eq_ppm)
{
	if(co2eq_ppm >= LVL_NORMAL && co2eq_ppm < LVL_MINIMUM)
	{
		return CO2_NORMAL;
	}
	else if (co2eq_ppm >= LVL_MINIMUM && co2eq_ppm < LVL_MEDIUM)
	{
		return CO2_MINIMUM;
	}
	else if (co2eq_ppm >= LVL_MEDIUM && co2eq_ppm < LVL_MAXIMUM)
	{
		return CO2_MEDIUM;
	}
	else if (co2eq_ppm >= LVL_MAXIMUM && co2eq_ppm < LVL_EXTREME)
	{
		return CO2_MAXIMUM;
	}
	else if (co2eq_ppm >= LVL_EXTREME)
	{
		return CO2_EXTREME;
	}
	else

	return CO2_UNDEFINED;
}

static int MovingAvg(int *ptrArrNumbers, long *ptrSum, int pos, int len, int nextNum)
{
  /*Subtract the oldest number from the previous sum, add the new number*/
  *ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;

  /*Assign the nextNum to the position in the array*/
  ptrArrNumbers[pos] = nextNum;

  /*Return the average*/
  return *ptrSum / len;
}

void Phrase_Repeat_Callback(TimerHandle_t xTimer)
{
	xTimerDelete(phrase_repeat, 100);
	phrase_repeat = NULL;
	co2_state_prev = CO2_UNDEFINED;
}

void Speech_Check_Callback(TimerHandle_t xTimer)
{
	xTimerDelete(speech_check, 100);
	speech_check = NULL;
	check = true;
}

