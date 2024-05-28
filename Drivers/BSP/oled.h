/*
 * OLED.h
 *
 *  Created on: May 17, 2024
 *      Author: 18330
 */

#ifndef BSP_OLED_H_
#define BSP_OLED_H_

/* 宏定义，用于配置OLED的SCL和SDA引脚 */
// OLED_W_SCL(x) 宏用于控制SCL引脚的状态，其中x为0或1，分别表示低电平和高电平
#define OLED_W_SCL(x)		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, x)
// OLED_W_SDA(x) 宏用于控制SDA引脚的状态，其中x为0或1，分别表示低电平和高电平
#define OLED_W_SDA(x)		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, x)


void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

#endif /* BSP_OLED_H_ */
