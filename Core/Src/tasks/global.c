/*
 * global.c
 *
 *  Created on: Oct 26, 2020
 *      Author: xqe2011
 */

#include "tasks/global.h"
#include "Library/Inc/rc.h"
#include "cmsis_os.h"
#include "Library/Inc/motor.h"
#include "Configurations/config.h"

static void Global_Task()
{
	const RC_Info* rc = RC_GetData();
	while(1) {
		/* 关电保护 */
		if (rc->sw2 == 2 || rc->state != RC_OK) {
			Motor_off();
		} else {
			Motor_on();
		}
		osDelay(10);
	}
}

/**
 * 全局任务初始化
 */
void Global_Init()
{
	static osThreadId_t globalTaskHandle;
	const osThreadAttr_t globalTask_attributes = {
	    .name = "globalTask",
	    .priority = (osPriority_t) osPriorityHigh,
	    .stack_size = 128 * 4
	 };
	globalTaskHandle = osThreadNew(Global_Task, NULL, &globalTask_attributes);
	UNUSED(globalTaskHandle);
}
