/*
 * main.h
 *
 *  Created on: 22. jul. 2016
 *      Author: vojis
 */

#ifndef MAIN_H_
#define MAIN_H_

#define RXBUFFERSIZE 100
#define True 1
#define False 0

/* Uncomment to get debug messages in the UART2 terminal*/
//#define DEBUG_MODE

#define MOTOR_PIN_X_1 GPIO_PIN_13
#define MOTOR_PIN_X_2 GPIO_PIN_14
#define MOTOR_PIN_X_3 GPIO_PIN_15
#define MOTOR_PIN_X_4 GPIO_PIN_9

#define MOTOR_PORT_X_1 GPIOC
#define MOTOR_PORT_X_2 GPIOC
#define MOTOR_PORT_X_3 GPIOC
#define MOTOR_PORT_X_4 GPIOC


#define MOTOR_PIN_Y_1 GPIO_PIN_2
#define MOTOR_PIN_Y_2 GPIO_PIN_3
#define MOTOR_PIN_Y_3 GPIO_PIN_5
#define MOTOR_PIN_Y_4 GPIO_PIN_11

#define MOTOR_PORT_Y_1 GPIOC
#define MOTOR_PORT_Y_2 GPIOC
#define MOTOR_PORT_Y_3 GPIOC
#define MOTOR_PORT_Y_4 GPIOC


#define MOTOR_PIN_Z_1 GPIO_PIN_1
#define MOTOR_PIN_Z_2 GPIO_PIN_2
#define MOTOR_PIN_Z_3 GPIO_PIN_0
#define MOTOR_PIN_Z_4 GPIO_PIN_4

#define MOTOR_PORT_Z_1 GPIOB
#define MOTOR_PORT_Z_2 GPIOB
#define MOTOR_PORT_Z_3 GPIOA
#define MOTOR_PORT_Z_4 GPIOA

void SystemClock_Config(void);
void Error_Handler(void);
void MX_GPIO_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void Init_motors(Stepper_t *stepper_x, Stepper_t *stepper_y, Stepper_t *stepper_z);
float fast_sqrt(float number);

#endif /* MAIN_H_ */
