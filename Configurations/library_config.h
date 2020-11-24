/*
 * library_config.h
 * 所有外设配置
 *
 *  Created on: Oct 19, 2020
 *      Author: xqe2011
 */

#ifndef LIBRARY_CONFIG_H_
#define LIBRARY_CONFIG_H_

#include "main.h"

/* 模块是否启用设置 */
#define CONFIG_IMU_ENABLE
#define CONFIG_MOTOR_ENABLE
#define CONFIG_RC_ENABLE
#define CONFIG_STORAGE_ENABLE
#define CONFIG_CAPACITY_ENABLE
#define CONFIG_DRIVER_CAN_ENABLE
#define CONFIG_DRIVER_FLASH_ENABLE
#define CONFIG_DRIVER_GPIO_ENABLE
#define CONFIG_DRIVER_SPI_ENABLE
#define CONFIG_DRIVER_UART_ENABLE
#define CONFIG_DRIVER_PWM_ENABLE
#define CONFIG_DRIVER_USB_ENABLE
#define CONFIG_REFEREE_ENABLE
#define CONFIG_DCT_ENABLE

#include "Library/Inc/drivers/can.h"
#include "Library/Inc/motor.h"

#ifdef CONFIG_DCT_ENABLE
	/* 是否启用PID显示	 */
	#define CONFIG_DCT_PID_ENABLE

	/* 是否启用日记输出 */
	#define CONFIG_DCT_LOG_ENABLE

	/* PID最大显示数量	 */
	#define CONFIG_DCT_PID_MAX_LENGTH 8

	/* DCT包最大大小(含SOF,CMDID等,但不包含末尾CRC8),不能大于0xFFFF */
	#define DCT_PACKET_MAX_LENGTH 512
#endif

/* CRC模块默认初始值 */
/* CRC8默认初始值 */
#define CONFIG_CRC_CRC8_DEFAULT_INIT_VALUE  0xFF

/* CRC8默认初始值 */
#define CONFIG_CRC_CRC16_DEFAULT_INIT_VALUE 0xFFFF

#ifdef CONFIG_REFEREE_ENABLE
	/* 裁判系统串口号 */
	#define CONFIG_REFEREE_UART_ID 1

	/* 裁判系统数据包最大长度 */
	#define CONFIG_REFEREE_DATA_MAX_LENGTH 200

	/* 裁判系统回调最大数量 */
	#define CONFIG_REFEREE_CALLBACKS_MAX 16

	/* 裁判系统队列深度 */
	#define CONFIG_REFEREE_MSGS_LENGTH 2048

#endif

#ifdef CONFIG_MOTOR_ENABLE
	/*
	 * 电机信息数组
	 * 如果是CAN电机,则需要有canNum参数, ID为电调ID
	 * 如果是PWM电机,则只需要ID, ID为PWM引脚ID
	 */
	#define CONFIG_MOTOR_INFOS {\
			{ canNum: CAN_1, id: 1, type: MOTOR_TYPE_RM3508 },\
			{ canNum: CAN_1, id: 2, type: MOTOR_TYPE_RM3508 },\
			{ canNum: CAN_1, id: 3, type: MOTOR_TYPE_RM3508 },\
			{ canNum: CAN_1, id: 4, type: MOTOR_TYPE_RM3508 },\
			{ canNum: CAN_1, id: 5, type: MOTOR_TYPE_RM6020 },\
			{ canNum: CAN_2, id: 6, type: MOTOR_TYPE_RM6020 },\
			{ 				 id: 1, type: MOTOR_TYPE_BLHEILS },\
			{ 				 id: 2, type: MOTOR_TYPE_BLHEILS },\
			{ canNum: CAN_1, id: 7, type: MOTOR_TYPE_RM2006 },\
	};

	/* CAN发送频率 */
	#define CONFIG_MOTOR_CAN_HZ 100
#endif

#ifdef CONFIG_DRIVER_UART_ENABLE
	extern UART_HandleTypeDef huart1;
	extern UART_HandleTypeDef huart3;

	/* 串口配置, bufferLength不可大于UART_BUFFER_MAX_LENGTH */
	#define CONFIG_UART_INFOS {\
			{ huart: &huart1, bufferLength: 18 },\
			{ huart: &huart3, bufferLength: 512 }\
	};

	/* 串口最大Buffer设置 */
	#define CONFIG_UART_BUFFER_MAX_LENGTH 512
#endif

#ifdef CONFIG_RC_ENABLE
	/* 遥控器UART编号 */
	#define CONFIG_RC_UART 0
#endif


#ifdef CONFIG_DRIVER_SPI_ENABLE
	extern SPI_HandleTypeDef hspi5;

	/* SPI配置 */
	#define CONFIG_SPI_INFOS {\
			{ hspi: &hspi5 }\
	};

#endif

#ifdef CONFIG_DRIVER_GPIO_ENABLE
	/* GPIO设置 */
	#define CONFIG_GPIO_INFOS {\
			{ pin: IMU_NSS_Pin, 		  port: IMU_NSS_GPIO_Port 			},\
			{ pin: POWER_1_Pin, 		  port: POWER_1_GPIO_Port 			},\
			{ pin: POWER_2_Pin, 		  port: POWER_2_GPIO_Port 			},\
			{ pin: POWER_3_Pin, 		  port: POWER_3_GPIO_Port 			},\
			{ pin: POWER_4_Pin, 	 	  port: POWER_4_GPIO_Port 			},\
	};
#endif

#ifdef CONFIG_DRIVER_PWM_ENABLE
	extern TIM_HandleTypeDef htim1;
	extern TIM_HandleTypeDef htim3;
	/* PWM输出引脚设置 */
	#define CONFIG_PWM_INFOS {\
			{ timer: &htim3, channel: TIM_CHANNEL_1 },\
			{ timer: &htim1, channel: TIM_CHANNEL_1 },\
			{ timer: &htim1, channel: TIM_CHANNEL_4 },\
	};
#endif

#ifdef CONFIG_IMU_ENABLE
	/* SPI编号 */
	#define CONFIG_IMU_SPI 0

	/* NSS的GPIO引脚ID */
	#define CONFIG_IMU_NSS_PIN 0

	/* 加热引脚的PWM引脚ID */
	#define CONFIG_IMU_HEAT_POWER_PIN 0

	/* 控制加速度计/磁力计的收敛速度的比例系数 */
	#define CONFIG_IMU_KP 2.0f

	/* 控制陀螺仪偏置的收敛速度的积分增益 */
	#define CONFIG_IMU_KI 0.01f

	/* 恒温PID参数 */
	#define CONFIG_PID_IMU_HEAT_P 1
	#define CONFIG_PID_IMU_HEAT_I 10
	#define CONFIG_PID_IMU_HEAT_D 0
	#define CONFIG_PID_IMU_HEAT_MAX_OUTPUT 70
	#define CONFIG_PID_IMU_HEAT_INTERGRAL_LIMIT 50
	#define CONFIG_PID_IMU_HEAT_DCT_ENABLE 0

	/* 恒温设定温度 */
	#define CONFIG_IMU_HEAT_TARGET_TEMP 50
#endif

#ifdef CONFIG_STORAGE_ENABLE
	/**
	 * 存储配置,如果删除/修改了存储块大小,请务必擦除FLASH存储扇区或整个FLASH
	 */
	#define CONFIG_STORAGE_INFOS {\
			{ size: 128 }\
	}
#endif

#ifdef CONFIG_DRIVER_FLASH_ENABLE
	/* FLASH存储扇区地址 */
	#define CONFIG_FLASH_SECTOR_ADDRESS 0x081E0000
	/* FLASH存储扇区大小 */
	#define CONFIG_FLASH_SECTOR_SIZE 128000
	/* FLASH存储扇区号 */
	#define CONFIG_FLASH_SECTOR_NUM FLASH_SECTOR_23
#endif

#ifdef CONFIG_CAPACITY_ENABLE
	/* 超级电容挂载CAN编号 */
	#define CONFIG_CAPACITY_CAN_NUM CAN_2

	/* CAN发送频率 */
	#define CONFIG_CAPACITY_HZ 1
#endif

#endif /* LIBRARY_CONFIG_H_ */
