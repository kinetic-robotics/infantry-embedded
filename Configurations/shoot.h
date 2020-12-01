/*
 * shoot.h
 *
 *  Created on: Nov 11, 2020
 *      Author: xqe2011
 */

#ifndef SHOOT_H_
#define SHOOT_H_

/* 任务是否启用设置 */
#define CONFIG_SHOOT_ENABLE

/* 云台LED闪烁延时,单位毫秒 */
#define CONFIG_SHOOT_LED_DELAY 100
/* 云台LED引脚编号 */
#define CONFIG_SHOOT_LED_PIN CONFIG_LED_PIN_3

/**
 * 电机ID配置,这里的ID并非是电调ID,而是电机信息数组的index
 * FRICTION_L: 左摩擦轮电机
 * FRICTION_R: 右摩擦轮电机
 * FIRE:       拨弹电机
 */
#define CONFIG_SHOOT_MOTOR_FRICTION_L 6
#define CONFIG_SHOOT_MOTOR_FRICTION_R 7
#define CONFIG_SHOOT_MOTOR_FIRE       8

/* 左摩擦轮正常运转时的电流 */
#define CONFIG_SHOOT_FRICTION_L_NORMAL_CURRENT   7500

/* 右摩擦轮正常运转时的电流 */
#define CONFIG_SHOOT_FRICTION_R_NORMAL_CURRENT   7500

/* 拨弹电机速度环 */
#define CONFIG_PID_SHOOT_FIRE_SPEED_P 1
#define CONFIG_PID_SHOOT_FIRE_SPEED_I 1
#define CONFIG_PID_SHOOT_FIRE_SPEED_D 0
#define CONFIG_PID_SHOOT_FIRE_SPEED_MAX_OUTPUT 3000
#define CONFIG_PID_SHOOT_FIRE_SPEED_INTERGRAL_LIMIT 1000

/* 拨弹电机连发模式下设定速度 */
#define CONFIG_SHOOT_FIRE_SPEED -1000

/* 拨弹电机卡弹判定速度 */
#define CONFIG_SHOOT_FIRE_STUCK_SPEED 10

/* 拨弹电机卡弹判定时间 */
#define CONFIG_SHOOT_FIRE_STUCK_TIME 1000

/* 拨弹电机卡弹反转速度 */
#define CONFIG_SHOOT_FIRE_STUCK_INVERSE_SPEED 1000

/* 拨弹电机卡弹判定时间 */
#define CONFIG_SHOOT_FIRE_STUCK_INVERSE_TIME 1000

#endif /* SHOOT_H_ */
