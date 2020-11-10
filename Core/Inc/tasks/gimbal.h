/*
 * gimbal.h
 *
 *  Created on: Oct 23, 2020
 *      Author: xqe2011
 */

#ifndef INC_TASKS_GIMBAL_H_
#define INC_TASKS_GIMBAL_H_

#include <stdint.h>

/**
 * 云台校准数据,需存储在STORAGE里
 */
typedef struct __packed {
	uint8_t flag;       /* 云台校准FLAG */
	int16_t yawOffset;  /* yaw轴中心偏移量,为编码器ecd值 */
	int16_t pitchOffset;/* pitch轴中心偏移量,为编码器ecd值 */
} Gimbal_CalibrationInfo;

/* 云台校准FLAG */
#define GIMBAL_CALIBRATION_FLAG_OK 1

/* 电机编号 */
#define GIMBAL_MOTOR_YAW 0
#define GIMBAL_MOTOR_PITCH 1

/* 云台状态机的Flag */
#define GIMBAL_FLAG_NOT_CALIBRATE 0
#define GIMBAL_FLAG_BACK_RUNNING 1
#define GIMBAL_FLAG_BACK_CALIBRATE 2
#define GIMBAL_FLAG_RUNNING 3

/* 云台校准信息的存储块ID */
#define GIMBAL_CALIBRATION_STORAGE_ID 0

void Gimbal_Init();

#endif /* INC_TASKS_GIMBAL_H_ */
