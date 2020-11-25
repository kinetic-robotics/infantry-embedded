/*
 * gimbal.c
 *
 *  Created on: Oct 23, 2020
 *      Author: xqe2011
 */


#include <math.h>
#include "cmsis_os.h"
#include "Configurations/config.h"
#include "tasks/gimbal.h"
#include "tasks/chassis.h"
#include "Library/Inc/storage.h"
#include "Library/Inc/imu.h"
#include "Library/Inc/tool.h"
#include "Library/Inc/rc.h"
#include "Library/Inc/motor.h"
#include "Library/Inc/led.h"
#include "Library/Inc/algorithm/pid.h"

/* library层接口 */
static const RC_Info* rc;
static const IMU_Info* imu;
/* 云台校准信息 */
static Gimbal_CalibrationInfo calibration;
static float offsetYawAngle, offsetPitchAngle; /* yaw和pitch轴启动并校准后时的朝向,单位度 */
/* 期望云台朝向,单位度 */
static float targetYawAngle, targetPitchAngle;
/* 电机信息 */
static const Motor_Info* motors[2];
static PID_Info motorSpeedPID[2];    /* 电机角速度环,单位为度/秒 */
static PID_Info motorPositionPID[2]; /* 电机角度环,单位为度 */
static PID_Info motorCalibratePID[2];/* 电机校准位置环,是电机编码器值 */
static int16_t motorsCurrent[2]; /* 电机电流 */

/**
 * 云台校准函数
 */
static void Gimbal_Calibrate()
{
	calibration.flag = GIMBAL_CALIBRATION_FLAG_OK;
	calibration.pitchOffset = motors[GIMBAL_MOTOR_PITCH]->ecd;
	calibration.yawOffset = motors[GIMBAL_MOTOR_YAW]->ecd;
	Storage_Write(GIMBAL_CALIBRATION_STORAGE_ID, &calibration, sizeof(calibration));
}

/**
 * 云台两轴闭环控制
 */
static void Gimbal_CloseLoopControl()
{
	float speedRef[2]; /* yaw和pitch轴的位置环输出 */
	float nowYawRelativeAngle;                                 /* yaw轴的相对中间偏移角度,单位度 */
	float nowPitchRelativeAngle = imu->pit - offsetPitchAngle; /* pitch轴的相对中间偏移角度,单位度 */
	/* 由于指南针存在0点,我们不希望过0点时选择一条较远的路径返回,所以进行路径选择 */
	nowYawRelativeAngle = imu->yaw - offsetYawAngle;
	float calcTargetYaw = targetYawAngle;
	if (nowYawRelativeAngle - targetYawAngle > 180) {
		calcTargetYaw = calcTargetYaw + 360;
	}else if(targetYawAngle - nowYawRelativeAngle > 180) {
		calcTargetYaw = calcTargetYaw - 360;
	}
	/* yaw轴预期速度计算，单位degree/s */
	speedRef[GIMBAL_MOTOR_YAW]       = PID_Calc(&motorPositionPID[GIMBAL_MOTOR_YAW], nowYawRelativeAngle, calcTargetYaw);
	/* yaw轴电机电流计算 */
	motorsCurrent[GIMBAL_MOTOR_YAW]  = PID_Calc(&motorSpeedPID[GIMBAL_MOTOR_YAW], imu->wz, speedRef[GIMBAL_MOTOR_YAW]);
	/* pitch轴预期速度计算，单位degree/s */
	speedRef[GIMBAL_MOTOR_PITCH]     = PID_Calc(&motorPositionPID[GIMBAL_MOTOR_PITCH], nowPitchRelativeAngle, targetPitchAngle);
	/* pitch轴电机电流计算,pitch电流是反的 */
	motorsCurrent[GIMBAL_MOTOR_PITCH]= -PID_Calc(&motorSpeedPID[GIMBAL_MOTOR_PITCH], imu->wy, speedRef[GIMBAL_MOTOR_PITCH]);
}

/**
 * 遥控器数据解析
 */
static void Gimbal_ParseRCData()
{
	targetYawAngle   += -rc->ch3 / RC_MAX_VALUE * CONFIG_GIMBAL_RC_YAW_SPEED;  /* 云台pitch增速 */
	targetPitchAngle +=  rc->ch4 / RC_MAX_VALUE * CONFIG_GIMBAL_RC_PITCH_SPEED;/* 云台yaw增速 */
	//TOOL_LIMIT(targetYawAngle, GIMBAL_RC_YAW_MIN_ANGLE, GIMBAL_RC_YAW_MAX_ANGLE);
	/* 遥控器信号超过+-360时处理一下 */
	if (targetYawAngle >= 360) {
		targetYawAngle = targetYawAngle - 360;
	} else if (targetYawAngle <= -360){
		targetYawAngle = targetYawAngle + 360;
	}
	TOOL_LIMIT(targetPitchAngle, CONFIG_GIMBAL_RC_PITCH_MIN_ANGLE, CONFIG_GIMBAL_RC_PITCH_MAX_ANGLE);
}

/**
 * 云台任务
 */
static void Gimbal_Task()
{
	uint8_t calibrateFlag = 0;
	uint8_t step = GIMBAL_FLAG_NOT_CALIBRATE; /* 云台状态Flag */
	uint16_t calcCount = 0; /* 云台校准稳定后计数,防止云台校准后马上读取imu的值飘动 */
	LED_BlinkInfo led; /* LED闪烁结构体 */
	/* 获取Library信息 */
	rc = RC_GetData();
	imu = IMU_GetData();
	/* 初始化LED */
	LED_BlinkInit(&led, CONFIG_GIMBAL_LED_PIN, CONFIG_GIMBAL_LED_DELAY);
	/* 获取电机信息 */
	motors[GIMBAL_MOTOR_YAW] = Motor_GetMotorData(CONFIG_GIMBAL_MOTOR_YAW);
	motors[GIMBAL_MOTOR_PITCH] = Motor_GetMotorData(CONFIG_GIMBAL_MOTOR_PITCH);
	/* 初始化PID */
	PID_CREATE_FROM_CONFIG(GIMBAL_SPEED_YAW, &motorSpeedPID[GIMBAL_MOTOR_YAW]);
	PID_CREATE_FROM_CONFIG(GIMBAL_SPEED_PITCH, &motorSpeedPID[GIMBAL_MOTOR_PITCH]);
	PID_CREATE_FROM_CONFIG(GIMBAL_POSITION_YAW, &motorPositionPID[GIMBAL_MOTOR_YAW]);
	PID_CREATE_FROM_CONFIG(GIMBAL_POSITION_PITCH, &motorPositionPID[GIMBAL_MOTOR_PITCH]);
	PID_CREATE_FROM_CONFIG(GIMBAL_CALIBRATE_YAW, &motorCalibratePID[GIMBAL_MOTOR_YAW]);
	PID_CREATE_FROM_CONFIG(GIMBAL_CALIBRATE_PITCH, &motorCalibratePID[GIMBAL_MOTOR_PITCH]);
	/* 读取云台校准信息 */
	Storage_Read(GIMBAL_CALIBRATION_STORAGE_ID, &calibration, sizeof(calibration));
	if (calibration.flag == GIMBAL_CALIBRATION_FLAG_OK) {
		step = GIMBAL_FLAG_BACK_RUNNING;
	}
	while(1) {
		/* 遥控器右开关上校准 */
		if (rc->sw2 == 1) {
			step = GIMBAL_FLAG_NOT_CALIBRATE;
			calibrateFlag = 1;
		} else if (calibrateFlag == 1){
			Gimbal_Calibrate();
			calibrateFlag = 0;
			step = GIMBAL_FLAG_BACK_RUNNING;
		}
		/* 云台回中,并IMU角度校准 */
		switch (step) {
			case GIMBAL_FLAG_NOT_CALIBRATE:
				motorsCurrent[GIMBAL_MOTOR_YAW] = 0;
				motorsCurrent[GIMBAL_MOTOR_PITCH] = 0;
				break;
			case GIMBAL_FLAG_BACK_RUNNING:
				/* 重置pitch和yaw轴pid */
				PID_Reset(&motorSpeedPID[GIMBAL_MOTOR_YAW]);
				PID_Reset(&motorSpeedPID[GIMBAL_MOTOR_PITCH]);
				PID_Reset(&motorPositionPID[GIMBAL_MOTOR_YAW]);
				PID_Reset(&motorPositionPID[GIMBAL_MOTOR_PITCH]);
				/* 更新底盘跟随云台模式的offset */
				Chassis_SetChassisFolloweYawEcd(calibration.yawOffset);
				/* 计算校准位置环PID */
				motorsCurrent[GIMBAL_MOTOR_YAW]  = PID_Calc(&motorCalibratePID[GIMBAL_MOTOR_YAW], motors[GIMBAL_MOTOR_YAW]->ecd, calibration.yawOffset);
				motorsCurrent[GIMBAL_MOTOR_PITCH]= PID_Calc(&motorCalibratePID[GIMBAL_MOTOR_PITCH], motors[GIMBAL_MOTOR_PITCH]->ecd, calibration.pitchOffset);
				if (fabs(motorCalibratePID[GIMBAL_MOTOR_YAW].err[PID_NOW]) < 22 && fabs(motorCalibratePID[GIMBAL_MOTOR_PITCH].err[PID_NOW]) < 22) {
					/* 等imu稳定 */
					if (++calcCount > 10) {
						motorsCurrent[GIMBAL_MOTOR_YAW] = 0;
						motorsCurrent[GIMBAL_MOTOR_PITCH] = 0;
						PID_Reset(&motorCalibratePID[GIMBAL_MOTOR_YAW]);
						PID_Reset(&motorCalibratePID[GIMBAL_MOTOR_PITCH]);
						step = GIMBAL_FLAG_BACK_CALIBRATE;
					}
				} else {
					calcCount = 0;
				}
				break;
			case GIMBAL_FLAG_BACK_CALIBRATE:
				offsetYawAngle = imu->yaw;
				offsetPitchAngle = imu->pit;
				step = GIMBAL_FLAG_RUNNING;
				break;
			case GIMBAL_FLAG_RUNNING:
				Gimbal_ParseRCData();
				Gimbal_CloseLoopControl();
				break;
		}
		/* 发送指令给电调 */
		Motor_Set(CONFIG_GIMBAL_MOTOR_YAW, motorsCurrent[GIMBAL_MOTOR_YAW]);
		Motor_Set(CONFIG_GIMBAL_MOTOR_PITCH, motorsCurrent[GIMBAL_MOTOR_PITCH]);
		/* 闪烁LED */
		LED_Blink(&led);
		osDelay(1);
	}
}

/**
 * 云台初始化
 */
void Gimbal_Init()
{
	#ifndef CONFIG_GIMBAL_ENABLE
		return;
	#endif
	static osThreadId_t gimbalTaskHandle;
	const osThreadAttr_t gimbalTask_attributes = {
			.name = "gimbalTask",
			.priority = (osPriority_t) osPriorityHigh,
			.stack_size = 128 * 4
	};
	gimbalTaskHandle = osThreadNew(Gimbal_Task, NULL, &gimbalTask_attributes);
	UNUSED(gimbalTaskHandle);
}
