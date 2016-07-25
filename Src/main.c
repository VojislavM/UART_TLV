/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "message.h"
#include "frame.h"
#include "AccelStepper.h"
#include <stdio.h>
#include "main.h"


#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


/* Struct FILE is implemented in stdio.h */
FILE __stdout;

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Private variables ---------------------------------------------------------*/
int test, i = 0;
int len, message_len = 0;

volatile uint8_t Rx_Buffer[100];
uint8_t Rx_data[2];
uint8_t tx_responce_buffer[1024];
int Rx_indx;
int Transfer_cplt;
char Rx_last[2] = {0, 0};


/* UART RX Interrupt callback routine */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	uint8_t i;
	/* Use selected UART for receive */
	if (huart->Instance == USART1){

		//TODO: check if this is necessary!!!
		/* Clear Rx_Buffer before receiving new data */
		if (Rx_indx==0){
			for (i=0;i<100;i++) Rx_Buffer[i]=0;
		}
		/* Start byte received */
		if(Rx_data[0] == FRAME_MARKER_START){
			/* Start byte received in the frame */
			if(Rx_last[0] == FRAME_MARKER_ESCAPE && Rx_Buffer[0] == FRAME_MARKER_START){
				Rx_Buffer[Rx_indx++]=Rx_data[0];
			}
			/* Real start byte received */
			else if(Rx_last[0] != FRAME_MARKER_ESCAPE){
				Rx_indx = 0;
				Rx_Buffer[Rx_indx++]=Rx_data[0];

			}
		}
		/* End byte received */
		else if(Rx_data[0] == FRAME_MARKER_END){
			/* End byte received in the frame */
			if(Rx_last[0] == FRAME_MARKER_ESCAPE && Rx_Buffer[0] == FRAME_MARKER_START){
				Rx_Buffer[Rx_indx++]=Rx_data[0];
			}
			/* Real end byte received */
			else if(Rx_last[0] != FRAME_MARKER_ESCAPE && Rx_Buffer[0] == FRAME_MARKER_START){
				Rx_Buffer[Rx_indx++]=Rx_data[0];
				message_len = Rx_indx;
				Rx_indx=0;
				/* Transfer complete, data is ready to read */
				Transfer_cplt=1;
				/* Disable USART1 interrupt */
				HAL_NVIC_DisableIRQ(USART1_IRQn);
			}
		}
		else{
			if(Rx_Buffer[0] == FRAME_MARKER_START){
				Rx_Buffer[Rx_indx++]=Rx_data[0];
			}
		}

		/* Store last received byte for ESC check */
		Rx_last[0] = Rx_data[0];
		/* Activate UART receive interrupt every time */
		HAL_UART_Receive_IT(&huart1, (uint8_t *)&Rx_data, 1);
	}
}


int main(void)
{
	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();

	/* Stepper motors struts */
	Stepper_t stepper_motor_x;
	Stepper_t stepper_motor_y;
	Stepper_t stepper_motor_z;

	/* Stepper motors initialization */
	Init_motors(&stepper_motor_x, &stepper_motor_y, &stepper_motor_z);

	/* Generate message - test message */
	message_t msg;
	message_init(&msg);
	message_tlv_add_command(&msg, COMMAND_MOVE_MOTOR);
	tlv_motor_position_t position = {10, 20, -15};
	message_tlv_add_motor_position(&msg, &position);
	message_tlv_add_checksum(&msg);

	/* Parsed message */
	message_t msg_parsed;
	tlv_command_t parsed_command;
	tlv_motor_position_t parsed_position;

	/* Response message */
	message_t msg_responce;


	uint8_t frame[1024];
	ssize_t frame_size;

#ifdef DEBUG_MODE
	printf("Hello\r\nKoruza driver terminal \r\n");
	printf("\n\n");
#endif

	/*Activate UART RX interrupt every time receiving 1 byte.*/
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&Rx_data, 1);

	/* Infinite loop */
	while(True){

		/* Move steppers. */
		run(&stepper_motor_x);
		run(&stepper_motor_y);
		run(&stepper_motor_z);

		test = 0;
		//frame received
		if (Transfer_cplt != 0){
#ifdef DEBUG_MODE
			printf("\nReceived serialized protocol message:\n");
			//HAL_UART_Transmit(&huart1, (uint8_t *)&Rx_Buffer, message_len, 1000);
			for (size_t i = 0; i < message_len; i++) {
				printf("%02X ", Rx_Buffer[i]);
			}
			printf("\n");
#endif

			//parse received message
			frame_parser((uint8_t *)&Rx_Buffer, message_len, &msg_parsed);
#ifdef DEBUG_MODE
			printf("\nParsed protocol message: ");
			message_print(&msg_parsed);
			printf("\n");
#endif
			//parse received message command
			if (message_tlv_get_command(&msg_parsed, &parsed_command) != MESSAGE_SUCCESS) {
#ifdef DEBUG_MODE
				printf("Failed to get command TLV.\n");
#endif
				message_free(&msg_parsed);
			}

			switch(parsed_command){
				case COMMAND_GET_STATUS:
					//Response message
					message_init(&msg_responce);
					message_tlv_add_reply(&msg_responce, REPLY_STATUS_REPORT);
					message_tlv_add_motor_position(&msg_responce, &position);
					message_tlv_add_checksum(&msg_responce);
#ifdef DEBUG_MODE
					printf("\n");
					printf("Parsed protocol message responce: ");
					message_print(&msg_responce);
					printf("\n");
#endif
					frame_size = frame_message(frame, sizeof(frame), &msg_responce);
					//send status message
					HAL_UART_Transmit(&huart1, (uint8_t *)&frame, frame_size, 1000);

					message_free(&msg_responce);
					break;

				case COMMAND_MOVE_MOTOR:
					break;
				case COMMAND_SEND_IR:
					break;
				case COMMAND_REBOOT:
					break;
				case COMMAND_FIRMWARE_UPGRADE:
					break;

			}

			/* Enable USART1 interrupt */
			HAL_NVIC_EnableIRQ(USART1_IRQn);
			/*Activate UART RX interrupt every time receiving 1 byte.*/
			HAL_UART_Receive_IT(&huart1, (uint8_t *)&Rx_data, 1);
			/* Reset transfer_complete flag */
			Transfer_cplt=0;

			//HAL_Delay(500);
			test++;

		}
		test = 1;
	}

}

/**Prototype for the printf() function**/
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

/** System Clock Configuration*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/100000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  /* SysTick_IRQn needs to be higher priority then other IRQs */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  //GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  /*
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
*/
  /*Configure GPIO pin : LD2_Pin */
  /*
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
*/
  /*Configure GPIO pin Output Level */
  /*
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
*/
}


void Init_motors(Stepper_t *stepper_x, Stepper_t *stepper_y, Stepper_t *stepper_z){

	/*## Initialize X axis stepper. ###*/
	InitStepper(stepper_x, HALF4WIRE, MOTOR_PIN_X_1, MOTOR_PORT_X_1, MOTOR_PIN_X_2, MOTOR_PORT_X_2, MOTOR_PIN_X_3, MOTOR_PORT_X_3, MOTOR_PIN_X_4, MOTOR_PORT_X_4, 1);
	setMaxSpeed(stepper_x, 500);
	setSpeed(stepper_x, 500);
	setAcceleration(stepper_x, 500);
	moveTo(stepper_x, -10000000);
	enableOutputs(stepper_x);

	/*## Initialize Y axis stepper. ###*/
	InitStepper(stepper_y, HALF4WIRE, MOTOR_PIN_Y_1, MOTOR_PORT_Y_1, MOTOR_PIN_Y_2, MOTOR_PORT_Y_2, MOTOR_PIN_Y_3, MOTOR_PORT_Y_3, MOTOR_PIN_Y_4, MOTOR_PORT_Y_4, 1);
	setMaxSpeed(stepper_y, 500);
	setSpeed(stepper_y, 500);
	setAcceleration(stepper_y, 500);
	moveTo(stepper_y, -10000000);
	enableOutputs(stepper_y);

	/*## Initialize Z axis stepper. ###*/
	InitStepper(stepper_z, HALF4WIRE, MOTOR_PIN_Z_1, MOTOR_PORT_Z_1, MOTOR_PIN_Z_2, MOTOR_PORT_Z_2, MOTOR_PIN_Z_3, MOTOR_PORT_Z_3, MOTOR_PIN_Z_4, MOTOR_PORT_Z_4, 1);
	setMaxSpeed(stepper_z, 500);
	setSpeed(stepper_z, 500);
	setAcceleration(stepper_z, 500);
	moveTo(stepper_z, -10000000);
	enableOutputs(stepper_z);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif
