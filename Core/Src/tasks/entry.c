/*
 * entry.c
 *
 *  Created on: Oct 23, 2020
 *      Author: xqe2011
 */

#include "cmsis_os.h"
#include "tasks/entry.h"
#include "tasks/chassis.h"
#include "tasks/global.h"
#include "tasks/gimbal.h"
#include "tasks/shoot.h"
#include "Library/Inc/library.h"
#include "Configurations/config.h"

/**
 * 启动任务
 */
void Entry_Task()
{
	Library_Init();
	Global_Init();
	Chassis_Init();
	Gimbal_Init();
	Shoot_Init();
	/* 为了防止通过本任务初始化的内存被释放,使用任务暂停而不是删除 */
	while (1) {
		vTaskSuspend(NULL);
	}

}
