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
#include "Library/Inc/library.h"
#include "Configurations/config.h"

/**
 * 启动任务
 */
void Entry_Task()
{
	Global_Init();
	Chassis_Init();
	Gimbal_Init();
}
