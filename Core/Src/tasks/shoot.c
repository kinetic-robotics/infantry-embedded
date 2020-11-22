/*
 * shoot.c
 *
 *  Created on: Nov 11, 2020
 *      Author: xqe2011
 */
#include "cmsis_os.h"
#include "Library/Inc/rc.h"
#include "Library/Inc/motor.h"
#include "Library/Inc/algorithm/pid.h"
#include "tasks/shoot.h"
#include "Configurations/config.h"

static const RC_Info* rc;				  /* 遥控器信息 */
static const Motor_Info* fireMotor; 	  /* 拨弹电机信息 */
static uint8_t frictionRCFlag = 0; 		  /* 摩擦轮电机遥控器开关flag */
static uint8_t isFrictionEnable = 0;	  /* 摩擦轮电机是否开启 */
static PID_Info fireSpeedPID;       	  /* 拨弹电机速度环PID */
static uint32_t fireStuckTime = 0;  	  /* 卡弹开始时间 */
static uint32_t fireStuckInverseTime = 0; /* 卡弹反转开始时间 */

/**
 * 摩擦轮控制函数
 */
static void Shoot_FrictionControl()
{
	/* 左开关向上打,再向中间打开启或关闭摩擦轮 */
	if (rc->sw1 == 1) {
		frictionRCFlag = 1;
	} else if (frictionRCFlag == 1) {
		/* 切换摩擦轮发射状态 */
		if (isFrictionEnable) {
			Motor_Set(CONFIG_SHOOT_MOTOR_FRICTION_L, 0);
			Motor_Set(CONFIG_SHOOT_MOTOR_FRICTION_R, 0);
			isFrictionEnable = 0;
		} else {
			Motor_Set(CONFIG_SHOOT_MOTOR_FRICTION_L, CONFIG_SHOOT_FRICTION_L_NORMAL_CURRENT);
			Motor_Set(CONFIG_SHOOT_MOTOR_FRICTION_R, CONFIG_SHOOT_FRICTION_R_NORMAL_CURRENT);
			isFrictionEnable = 1;
		}
		frictionRCFlag = 0;
	}
}

/**
 * 拨弹电机控制函数
 */
static void Shoot_FireControl()
{
	float setSpeed;       /* 拨弹电机设定速度 */
	uint8_t isEnable = 0; /* 拨弹电机是否开启 */
	/* 左开关向下打开拨弹电机 */
	if (rc->sw1 == 2) {
		/* 手动开启拨弹电机连发模式 */
		setSpeed = CONFIG_SHOOT_FIRE_SPEED;
		isEnable = 1;
	} else {
		setSpeed = 0;
	}
	/* 防卡弹处理 */
	if (isEnable && fireMotor->speedRpm < CONFIG_SHOOT_FIRE_STUCK_SPEED && fireMotor->state && setSpeed != 0) {
		if (fireStuckTime == 0) {
			fireStuckTime = xTaskGetTickCount();
		}
		if (xTaskGetTickCount() - fireStuckTime > CONFIG_SHOOT_FIRE_STUCK_TIME) {
			if (fireStuckInverseTime == 0) {
				fireStuckInverseTime = xTaskGetTickCount();
			}
			/* 卡弹反转 */
			if (xTaskGetTickCount() - fireStuckInverseTime < CONFIG_SHOOT_FIRE_STUCK_INVERSE_TIME) {
				setSpeed = CONFIG_SHOOT_FIRE_STUCK_INVERSE_SPEED;
			} else {
				setSpeed = 0;
				fireStuckTime = 0;
				fireStuckInverseTime = 0;
			}
		}
	} else {
		fireStuckTime = 0;
		fireStuckInverseTime = 0;
	}
	Motor_Set(CONFIG_SHOOT_MOTOR_FIRE, PID_Calc(&fireSpeedPID, fireMotor->speedRpm, setSpeed));
}

/**
 * 射击任务
 */
static void Shoot_Task()
{
	/* 获取遥控器信息 */
	rc = RC_GetData();
	/* 获取拨弹电机信息 */
	fireMotor = Motor_GetMotorData(CONFIG_SHOOT_MOTOR_FIRE);
	/* 初始化PID */
	PID_CREATE_FROM_CONFIG(SHOOT_FIRE_SPEED, &fireSpeedPID);
	/* 上电关摩擦轮 */
	Motor_Set(CONFIG_SHOOT_MOTOR_FRICTION_L, 0);
	Motor_Set(CONFIG_SHOOT_MOTOR_FRICTION_R, 0);
	while(1) {
		Shoot_FrictionControl();
		Shoot_FireControl();
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
