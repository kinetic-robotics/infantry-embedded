/*
 * chassis.h
 *
 *  Created on: Oct 23, 2020
 *      Author: xqe2011
 */

#ifndef INC_TASKS_CHASSIS_H_
#define INC_TASKS_CHASSIS_H_

/* 底盘控制模式 */
#define CHASSIS_MODE_CHASSIS_FOLLOW 0
#define CHASSIS_MODE_NO 1

void Chassis_Init();
void Chassis_SetChassisFolloweYawEcd(uint32_t ecd);

#endif /* INC_TASKS_CHASSIS_H_ */
