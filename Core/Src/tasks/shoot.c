/*
 * shoot.c
 *
 *  Created on: Nov 11, 2020
 *      Author: xqe2011
 */
#include "cmsis_os.h"
#include "tasks/shoot.h"
#include "Configurations/config.h"

/**
 * 射击任务
 */
static void Shoot_Task()
{
	while(1) {
		osDelay(10);
	}
}


/**
 * 射击初始化
 */
void Shoot_Init()
{
	#ifndef CONFIG_SHOOT_ENABLE
		return;
	#endif
	static osThreadId_t shootTaskHandle;
	const osThreadAttr_t shootTaskAttributes = {
			.name = "shootTask",
			.priority = (osPriority_t) osPriorityHigh,
			.stack_size = 128 * 4
	};
	shootTaskHandle = osThreadNew(Shoot_Task, NULL, &shootTaskAttributes);
	UNUSED(shootTaskHandle);
}
