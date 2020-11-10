/***************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

/************************** (C) COPYRIGHT 2018 DJI *************************
 * @update
 * @history
 * Version     Date              Author           Modification
 * V1.0.0      January-15-2018   ric.luo
 * @verbatim
 ************************** (C) COPYRIGHT 2018 DJI *************************/

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "cmsis_os.h"
#include "Configurations/config.h"
#include "tasks/chassis.h"
#include "tasks/gimbal.h"
#include "Configurations/config.h"
#include "Library/Inc/tool.h"
#include "Library/Inc/rc.h"
#include "Library/Inc/algorithm/pid.h"

static float globalVx = 0; /* 底盘前后速度,该速度会与用户输入相加 */
static float globalVy = 0; /* 底盘左右速度,该速度会与用户输入相加 */
static float globalVw = 0; /* 底盘旋转速度,该速度会与用户输入相加 */
static PID_Info wheelsSpeedPID[4]; /* 底盘速度环 */
static PID_Info chassisFollowPID;  /* 底盘跟随云台位置环 */
static uint8_t controlMode = CHASSIS_MODE_CHASSIS_FOLLOW; /* 底盘控制模式,可取值为CHASSIS_MODE_ */
static uint16_t chassisFolloweYawEcd; /* 底盘跟随云台时的yaw角度 */
static const Motor_Info* yawMotor; /* yaw轴电机信息 */
const RC_Info* rc; /* 遥控器信息 */

/**
 * 底盘的运动分解计算
 * @param vx 左右平移速度(mm/s)，右为正方向
 * @param vy 前后平移速度(mm/s)，前为正方向
 * @param vw 底盘旋转速度(degree/s)，逆时针为正方向
 * @param speed 4个电机转速(rpm)数组
 * @note  下面是电机轮子编号，左上角为0号
 * @map   1 %++++++% 0
              ++++
              ++++
          2 %++++++% 3
 */
static void Chassis_SpeedCalc(float vx, float vy, float vw, int16_t speed[])
{
  static float rotateRatioF = ((CONFIG_CHASSIS_WHEELBASE + CONFIG_CHASSIS_WHEELTRACK)/2.0f - CONFIG_CHASSIS_GIMBAL_OFFSET)/TOOL_RADIAN_COEF;
  static float rotateRatioB = ((CONFIG_CHASSIS_WHEELBASE + CONFIG_CHASSIS_WHEELTRACK)/2.0f + CONFIG_CHASSIS_GIMBAL_OFFSET)/TOOL_RADIAN_COEF;
  static float wheelRPMRatio = 60.0f/(CONFIG_CHASSIS_PERIMETER * CONFIG_CHASSIS_DECELE_RATIO);

  int16_t wheelRPM[4];
  float max = 0;

  TOOL_ABS_LIMIT(vx, CONFIG_CHASSIS_MAX_VX_SPEED);  //mm/s
  TOOL_ABS_LIMIT(vy, CONFIG_CHASSIS_MAX_VY_SPEED);  //mm/s
  TOOL_ABS_LIMIT(vw, CONFIG_CHASSIS_MAX_VR_SPEED);  //deg/s

  wheelRPM[0] = (+vx - vy + vw * rotateRatioF) * wheelRPMRatio;
  wheelRPM[1] = (+vx + vy + vw * rotateRatioF) * wheelRPMRatio;
  wheelRPM[2] = (-vx + vy + vw * rotateRatioB) * wheelRPMRatio;
  wheelRPM[3] = (-vx - vy + vw * rotateRatioB) * wheelRPMRatio;

  //find max item
  for (size_t i = 0; i < 4; i++)
  {
    if (abs(wheelRPM[i]) > max)
      max = abs(wheelRPM[i]);
  }
  //equal proportion
  if (max > CONFIG_CHASSIS_MAX_WHEEL_RPM)
  {
    float rate = CONFIG_CHASSIS_MAX_WHEEL_RPM / max;
    for (size_t i = 0; i < 4; i++)
      wheelRPM[i] *= rate;
  }
  memcpy(speed, wheelRPM, 4*sizeof(int16_t));
}

/**
 * 底盘跟随云台,小陀螺模式控制
 */
static void Chassis_GimbalControl()
{
	/* 小陀螺启动 */
	if (rc->wheel != 0) {
		controlMode = CHASSIS_MODE_NO;
	} else  {
		controlMode = CHASSIS_MODE_CHASSIS_FOLLOW;
	}
	float nowEcd, targetEcd;
	switch(controlMode) {
		case CHASSIS_MODE_CHASSIS_FOLLOW:
			nowEcd = yawMotor->ecd;
			targetEcd = chassisFolloweYawEcd;
			/* 由于存在0点,我们不希望过0点时选择一条较远的路径返回,所以进行路径选择 */
			if (nowEcd - chassisFolloweYawEcd > CONFIG_GIMBAL_YAW_ECD / 2) {
				targetEcd = targetEcd + CONFIG_GIMBAL_YAW_ECD;
			}else if(chassisFolloweYawEcd - nowEcd > CONFIG_GIMBAL_YAW_ECD / 2) {
				targetEcd = targetEcd - CONFIG_GIMBAL_YAW_ECD;
			}
			globalVw = PID_Calc(&chassisFollowPID, nowEcd, targetEcd);
			break;
		case CHASSIS_MODE_NO:
			globalVw = 0;
			PID_Reset(&chassisFollowPID);
			break;
	}
}

/**
 * 设置底盘跟随云台模式时需要保持的yaw轴电机角度
 */
void Chassis_SetChassisFolloweYawEcd(uint32_t ecd)
{
	chassisFolloweYawEcd = ecd;
}

/**
 * 底盘任务
 */
static void Chassis_Task()
{
  int16_t motorsSpeedTarget[4]; /* 底盘电机期望转速(rpm) */
  int16_t motorsCurrent[4]; /* 底盘电机电流 */
  const uint8_t motorsID[4] = {
		  CONFIG_CHASSIS_MOTOR_RF,
		  CONFIG_CHASSIS_MOTOR_LF,
		  CONFIG_CHASSIS_MOTOR_LB,
		  CONFIG_CHASSIS_MOTOR_RB
  };
  const Motor_Info* motors[4];
  /* 获取电机信息 */
  for (size_t i = 0; i < 4; i++) {
	  motors[i] = Motor_GetMotorData(motorsID[i]);
  }
  /* 获取遥控器信息 */
  rc = RC_GetData();
  yawMotor = Motor_GetMotorData(CONFIG_CHASSIS_MOTOR_GIMBAL_YAW);
  while(1) {
	/* 小陀螺,底盘跟随云台控制 */
	Chassis_GimbalControl();
    /* 底盘控制信号获取,遥控器对底盘的控制信息转化为标准单位，平移为(mm/s)旋转为(degree/s), 同时加入全局速度 */
    float vx = rc->ch1 / RC_MAX_VALUE * CONFIG_CHASSIS_MAX_VX_SPEED + globalVx;         /* 底盘前后速度 */
    float vy = rc->ch2 / RC_MAX_VALUE * CONFIG_CHASSIS_MAX_VY_SPEED + globalVy;         /* 底盘左右速度 */
    float vw = rc->wheel / RC_WHEEL_MAX_VALUE * CONFIG_CHASSIS_MAX_VR_SPEED + globalVw; /* 底盘旋转速度 */

    /* 底盘速度分解，计算底盘电机转速 */
    Chassis_SpeedCalc(vx, vy, vw, motorsSpeedTarget);

    /* 闭环PID控制,并将计算好的电流值发送给电调 */
    for (size_t i = 0; i < 4; i++) {
      motorsCurrent[i] = PID_Calc(&wheelsSpeedPID[i], motors[i]->speedRpm, motorsSpeedTarget[i]);
      Motor_Set(motorsID[i], motorsCurrent[i]);
    }
    osDelay(10);
  }
}

/**
 * 底盘初始化
 */
void Chassis_Init()
{
  /* 初始化PID */
  PID_CREATE_FROM_CONFIG(CHASSIS_RF, &wheelsSpeedPID[0]);
  PID_CREATE_FROM_CONFIG(CHASSIS_LF, &wheelsSpeedPID[1]);
  PID_CREATE_FROM_CONFIG(CHASSIS_LB, &wheelsSpeedPID[2]);
  PID_CREATE_FROM_CONFIG(CHASSIS_RB, &wheelsSpeedPID[3]);
  PID_CREATE_FROM_CONFIG(CHASSIS_FOLLOW, &chassisFollowPID);
  static osThreadId_t chassisTaskHandle;
  const osThreadAttr_t chassisTask_attributes = {
    .name = "chassisTask",
    .priority = (osPriority_t) osPriorityHigh,
    .stack_size = 128 * 4
  };
  chassisTaskHandle = osThreadNew(Chassis_Task, NULL, &chassisTask_attributes);
  UNUSED(chassisTaskHandle);
}
