/*
 * main_task.h
 *
 *  Created on: 2021-04-29
 *      Author: GDR
 */

#ifndef MAIN_TASK_H_
#define MAIN_TASK_H_

#define LVL_NORMAL		0		/*0 - 1000 Concentrations typical of occupied indoor spaces with good air exchange.*/
#define LVL_MINIMUM		1000	/*1000 - 2000 Complaints of drowsiness and poor air.*/
#define LVL_MEDIUM		2000	/*2000 - 5000 Headaches, sleepiness and stagnant, stale, stuffy air. Poor concentration, loss of attention, increased heart rate and slight nausea may also be present.*/
#define LVL_MAXIMUM		5000	/*5000 Workplace exposure limit (as 8-hour TWA) in most jurisdictions.*/
#define LVL_EXTREME		40000	/*40000 Exposure may lead to serious oxygen deprivation resulting in permanent brain damage, coma, even death.*/

#define GREETING_PHRASE		0
#define CO2_NORMAL_PHRASE	1
#define CO2_MINIMUM_PHRASE	2
#define CO2_MEDIUM_PHRASE	3
#define CO2_MAXIMUM_PHRASE	4
#define CO2_EXTREME_PHRASE	5

#define CO2_MEDIUM_REPEAT	60000U
#define CO2_MAXIMUM_REPEAT	30000U
#define CO2_EXTREME_REPEAT	15000U
#define SPEECH_ENGINE_CHECK	5000U

typedef enum co2status
{
	CO2_NORMAL,
	CO2_MINIMUM,
	CO2_MEDIUM,
	CO2_MAXIMUM,
	CO2_EXTREME,
	CO2_UNDEFINED
}co2status_t;

extern TaskHandle_t main_task_handle;
void main_task(void *param);

#endif /* MAIN_TASK_H_ */
