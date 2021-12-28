/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f1xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
int esp8266_mode = 1; // esp8266_mode = 1 -> server mode, esp8266_mode = 0 -> client mode
int connect_flag = 0; //connect_flag = 0 -> not connected, 1-> TCP not connected but wifi connected, 2-> connected
int sending_flag = 0; //set the sending flag, 1-> is sending, 0 -> is not sending
int link_number = -1; //表示连接序号, 0号连接可client或server连接,其他id只能用于连接远程server,-1表示没有连接
int timer_count = 0;
int connection_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void send_cmd(uint8_t*, int); // send command with delay
void send_message(uint8_t*); //send message to other client
void send_msg_uart1(uint8_t *msg, int delay_time);

void init_var();
void init_server();
void connect_to_wifi();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
extern uint8_t uart1_rx_buffer[2048]; // for uart1 receive buffer
extern uint8_t uart2_rx_buffer[2048]; // for uart2 receive buffer

extern unsigned char received_Data[2048];
extern unsigned char received_Length;
extern int complete_recv;
extern int mode;
extern int time_limit;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) {
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */

	/* USER CODE END NonMaskableInt_IRQn 0 */
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */

	/* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void) {
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_HardFault_IRQn 0 */
		/* USER CODE END W1_HardFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void) {
	/* USER CODE BEGIN MemoryManagement_IRQn 0 */

	/* USER CODE END MemoryManagement_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
		/* USER CODE END W1_MemoryManagement_IRQn 0 */
	}
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
void BusFault_Handler(void) {
	/* USER CODE BEGIN BusFault_IRQn 0 */

	/* USER CODE END BusFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_BusFault_IRQn 0 */
		/* USER CODE END W1_BusFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void) {
	/* USER CODE BEGIN UsageFault_IRQn 0 */

	/* USER CODE END UsageFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_UsageFault_IRQn 0 */
		/* USER CODE END W1_UsageFault_IRQn 0 */
	}
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void) {
	/* USER CODE BEGIN SVCall_IRQn 0 */

	/* USER CODE END SVCall_IRQn 0 */
	/* USER CODE BEGIN SVCall_IRQn 1 */

	/* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {
	/* USER CODE BEGIN DebugMonitor_IRQn 0 */

	/* USER CODE END DebugMonitor_IRQn 0 */
	/* USER CODE BEGIN DebugMonitor_IRQn 1 */

	/* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void) {
	/* USER CODE BEGIN PendSV_IRQn 0 */

	/* USER CODE END PendSV_IRQn 0 */
	/* USER CODE BEGIN PendSV_IRQn 1 */

	/* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void) {
	/* USER CODE BEGIN SysTick_IRQn 0 */

	/* USER CODE END SysTick_IRQn 0 */
	HAL_IncTick();
	/* USER CODE BEGIN SysTick_IRQn 1 */

	/* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles DMA1 channel5 global interrupt.
 */
void DMA1_Channel5_IRQHandler(void) {
	/* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

	/* USER CODE END DMA1_Channel5_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_usart1_rx);
	/* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

	/* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
 * @brief This function handles DMA1 channel6 global interrupt.
 */
void DMA1_Channel6_IRQHandler(void) {
	/* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

	/* USER CODE END DMA1_Channel6_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_usart2_rx);
	/* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

	/* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
 * @brief This function handles EXTI line[9:5] interrupts.
 */
void EXTI9_5_IRQHandler(void) {
	/* USER CODE BEGIN EXTI9_5_IRQn 0 */

	/* USER CODE END EXTI9_5_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
	/* USER CODE BEGIN EXTI9_5_IRQn 1 */

	/* USER CODE END EXTI9_5_IRQn 1 */
}

/**
 * @brief This function handles TIM2 global interrupt.
 */
void TIM2_IRQHandler(void) {
	/* USER CODE BEGIN TIM2_IRQn 0 */

	/* USER CODE END TIM2_IRQn 0 */
	HAL_TIM_IRQHandler(&htim2);
	/* USER CODE BEGIN TIM2_IRQn 1 */

	/* USER CODE END TIM2_IRQn 1 */
}

/**
 * @brief This function handles TIM3 global interrupt.
 */
void TIM3_IRQHandler(void) {
	/* USER CODE BEGIN TIM3_IRQn 0 */

	/* USER CODE END TIM3_IRQn 0 */
	HAL_TIM_IRQHandler(&htim3);
	/* USER CODE BEGIN TIM3_IRQn 1 */
	if (time_limit > 0) {
		time_limit--;
		Display_Time(time_limit);
	} else {
		HAL_TIM_Base_Stop_IT(&htim3);
		mode = 2;
	}
	/* USER CODE END TIM3_IRQn 1 */
}

/**
 * @brief This function handles USART1 global interrupt.
 */
void USART1_IRQHandler(void) {
	/* USER CODE BEGIN USART1_IRQn 0 */

	/* USER CODE END USART1_IRQn 0 */
	HAL_UART_IRQHandler(&huart1);
	/* USER CODE BEGIN USART1_IRQn 1 */
	HAL_UART_RxCpltCallback(&huart1);
	HAL_UART_Receive_DMA(&huart1, (uint8_t*) uart1_rx_buffer, 2048);
	/* USER CODE END USART1_IRQn 1 */
}

/**
 * @brief This function handles USART2 global interrupt.
 */
void USART2_IRQHandler(void) {
	/* USER CODE BEGIN USART2_IRQn 0 */

	/* USER CODE END USART2_IRQn 0 */
	HAL_UART_IRQHandler(&huart2);
	/* USER CODE BEGIN USART2_IRQn 1 */
	HAL_UART_RxCpltCallback(&huart2);
	HAL_UART_Receive_DMA(&huart2, (uint8_t*) uart2_rx_buffer, 2048);
	/* USER CODE END USART2_IRQn 1 */
}

/**
 * @brief This function handles EXTI line[15:10] interrupts.
 */
void EXTI15_10_IRQHandler(void) {
	/* USER CODE BEGIN EXTI15_10_IRQn 0 */

	/* USER CODE END EXTI15_10_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
	/* USER CODE BEGIN EXTI15_10_IRQn 1 */

	/* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	HAL_Delay(100);
//    if (esp8266_mode == 1) {// for the server mode
//        switch (GPIO_Pin) {
//            case KEY0_Pin:
//                if (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET) {
//                    reset();
//                    init_server();
//                }
//                break;
//            case KEY1_Pin: // to disconnect
//                if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
//                    send_cmd((uint8_t *) "AT+CIPCLOSE=0\r\n", 2000);
//                    init_var();
//                }
//                break;
//            default:
//                break;
//        }
//    } else {//for the client mode
//        switch (GPIO_Pin) {
//            case KEY0_Pin:
//                if (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET) {
//                    reset();
//                    init_var();
//                    send_cmd(sta_cmd, 2000);
//                    send_cmd(restart, 5000);
//                    HAL_Delay(10000);//WAIT FOR CONNECTION
//                    int i = 0;
//                    uint8_t msg[100];
//                    while (connect_flag != 1 && i <= 3) {
//                    	sprintf(msg, "connect_flag=%d\r\n", connect_flag);
//                    	send_msg_uart1(msg, 2000);
////                        connect_flag = 0;
//                        if (i >= 3) {
//                            send_msg_uart1(conn_fail_msg, 0);
//                            send_msg_uart1(split, 0);
//                            HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);//0
//                            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);//0
//                            connect_flag = 0;
//                            return;
//                        }
//                        send_cmd(join_ap, 10000);
//                        ++i;
//                    }
//                    send_cmd(set_mux, 5000);
//                    send_cmd(server_set_server, 5000);
//                    send_cmd(start_tcp, 5000);
//                    send_cmd(look_up_cmd, 5000);
//                    break;
//                }
//            case KEY1_Pin:
//                if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
//                    send_cmd(quit_ap, 2000);
//                    init_var();
//                    break;
//                }
//            default:
//                break;
//        }
//    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == (&htim3)) {
		HAL_Delay(300);
		++timer_count;
		timer_count %= 15;
//		if (timer_count == 0 && connect_flag == 1) {
//			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//			if (esp8266_mode == 1) { //server mode
//				send_cmd((uint8_t*) "AT+PING=\"192.168.4.2\"\r\n", 0);
//			} else {
//				send_cmd((uint8_t*) "AT+PING=\"192.168.4.1\"\r\n", 0);
//			}
//
//		}
		if (connect_flag == 0) {
			HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET); //0
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); //0
			return;
		}
		if (connect_flag == 1) { // 连接上wifi 但是没有连接tcp
			HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET); //1
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); //0
			return;
		}
		if (connect_flag == 2) {
			if (sending_flag == 0) {
				HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET); //0
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); //1
			} else if (sending_flag == 1) {
				HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET); //1
				HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET); //1
			}
			return;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	// to receive the data from the usart1
	if (huart->Instance == USART1) {
		if (RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) {
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);
			HAL_UART_DMAStop(&huart1);
			uint8_t data_length = 2048 - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
			send_msg_uart1((uint8_t*) "enter usart receive function", 0);
			if (link_number != -1 && uart1_rx_buffer[0] == '>'
					&& connect_flag == 2) {
				sending_flag = 1;
				HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET); //1
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); //1
				send_message(uart1_rx_buffer + 1);
				sending_flag = 0;
				HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET); //0
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); //1
			} else if (strcmp(uart1_rx_buffer, "server\r\n") == 0) {
				esp8266_mode = 1;
				send_msg_uart1((uint8_t*) "SET AS SERVER\r\n", 0);
				send_cmd("AT+CWMODE=3\r\n", 1000);
			} else if (strcmp(uart1_rx_buffer, "client\r\n") == 0) {
				esp8266_mode = 0;
				send_msg_uart1((uint8_t*) "SET AS CLIENT\r\n", 0);
				send_cmd("AT+CWJAP?\r\n", 1000);
				send_cmd("AT+CIFSR\r\n", 1000);
			} else if (strcmp(uart1_rx_buffer, "start\r\n") == 0
					&& esp8266_mode == 1) {
				//reset();
				init_server();
			} else if (strcmp(uart1_rx_buffer, "connect to wifi\r\n") == 0
					&& esp8266_mode == 0) {
				//reset();
				connect_to_wifi();
			} else if (strcmp(uart1_rx_buffer, "connect to server\r\n") == 0
					&& esp8266_mode == 0) {
				send_cmd(
						(uint8_t*) "AT+CIPSTART=0,\"TCP\",\"192.168.4.1\",8089\r\n",
						0);
			} else if (strcmp(uart1_rx_buffer, "close connection\r\n") == 0) {
				send_cmd((uint8_t*) "AT+CIPCLOSE=0\r\n", 2000);
				init_var();
			} else if (strncmp(uart1_rx_buffer, "quit wifi\r\n") == 0
					&& esp8266_mode == 0) {
				send_cmd("AT+CWQAP\r\n", 1000);
				init_var();
			} else if (strncmp(uart1_rx_buffer, "ip status\r\n") == 0) {
				send_cmd("AT+CIPSTATUS\r\n", 1000);
				send_cmd("AT+CIFSR\r\n", 1000);
			} else if (strncmp(uart1_rx_buffer, "wifi status\r\n") == 0) {
//				send_cmd("AT+CWSAP?\r\n", 1000);
				send_cmd("AT+CWLIF\r\n", 1000);
				send_cmd("AT+CIPSTATUS\r\n", 1000);
			} else {
				HAL_UART_Transmit(&huart2, (uint8_t*) uart1_rx_buffer,
						data_length, 0xffff);
			}
			memset(uart1_rx_buffer, 0, data_length);
			data_length = 0;
			HAL_UART_Receive_DMA(&huart1, (uint8_t*) uart1_rx_buffer, 2048);
		}
	} else if (huart->Instance == USART2) { // to receive the data from the usart2
		if (RESET != __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE)) {
			__HAL_UART_CLEAR_IDLEFLAG(&huart2);
			HAL_UART_DMAStop(&huart2);
			uint8_t data_length = 2048 - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
			HAL_UART_Transmit(&huart1, (uint8_t*) "\r\nreceive usarrt2\r\n", 20, 0xffff);
			HAL_UART_Transmit(&huart1, (uint8_t*)uart2_rx_buffer, strlen(uart2_rx_buffer), 0xffff);
			if (strncmp(uart2_rx_buffer, "WIFI CONNECTED\r\n", 16) == 0
					&& connect_flag == 0) {
				connect_flag = 1;
				HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET); //0
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); //1
				link_number = 0;
			} else if (strncmp(uart2_rx_buffer, "WIFI GOT IP\r\n", 13) == 0) { // client 端会收到这个消息, 这里变成连接状�??
				connect_flag = 1;
				HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET); //0
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); //1
				link_number = 0;
			} else if (connect_flag == 1
					&& strncmp(uart2_rx_buffer, (uint8_t*) "+", 1) == 0) {
				if (strstr(uart2_rx_buffer, "+timeout") != NULL) {
					if (connection_counter > 2) {
						send_msg_uart1((uint8_t*) "connection closed\r\n", 100);
						send_cmd((uint8_t*) "AT+CWQAP\r\n", 2000);
						init_var();
						connection_counter = 0;
						//StateChange(0);
					} else {
						send_msg_uart1((uint8_t*) "connection disturbance\r\n",
								100);
						++connection_counter;
					}
				} else {
					connection_counter = 0;
//                        send_msg_uart1((uint8_t *) "connection alive\r\n", 100);
				}
			} else if (connect_flag == 2) {
				if (strcmp(uart2_rx_buffer, "WIFI DISCONNECT\r\n", 16) == 0) { //client 端会收到这个消息
					init_var();
					HAL_UART_Transmit(&huart1, (uint8_t*) uart2_rx_buffer,
							data_length, 0xffff);
				} else if (strncmp(uart2_rx_buffer, "\r\n+IPD,", 7) == 0) {
					HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET); //1
					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); //1
					HAL_UART_Transmit(&huart1, (uint8_t*) uart2_rx_buffer,
							data_length, 0xffff);

					int i, t;
					for (i = 0; i < data_length; i++) {
						if (strcmp(':', uart2_rx_buffer[i]) == 0) {
							t = i + 1;
							break;
						}
					}
					for (i = 0; i < data_length - t; i++) {
						received_Data[i] = uart2_rx_buffer[i + t];
					}
					complete_recv = 1;
					received_Length = data_length;

					if (strstr(received_Data,"Q:")!=NULL) {
						//HAL_UART_Transmit(&huart1, (uint8_t*) test, 20, 0xffff);
						HAL_UART_Transmit(&huart1, (uint8_t*) received_Data,
								received_Length, 0xffff);
						Show_Question();
						HAL_TIM_Base_Start_IT(&htim3);
						mode = 1;
						complete_recv = 0;
					} else if (strstr(received_Data,"F:")!=NULL) {
						HAL_UART_Transmit(&huart1, (uint8_t*) received_Data,
								received_Length, 0xffff);
						Show_Feedback();
						mode = 0;
						complete_recv = 0;
						HAL_TIM_Base_Stop_IT(&htim3);
					}

//					HAL_UART_Transmit(&huart1, (uint8_t*) received_Data,
//											received_Length, 0xffff);

//					printOut(uart2_rx_buffer + 1 + idx, data_length - idx, 2,
//							connect_flag);
				} else if (strncmp(uart2_rx_buffer, (uint8_t*) "SEND FAIL", 9)
						== 0) {
					init_var();
					HAL_UART_Transmit(&huart1, (uint8_t*) uart2_rx_buffer,
							data_length, 0xffff);
				} else if (strncmp(uart2_rx_buffer, (uint8_t*) "0,CLOSE OK", 10)
						== 0) {
					connect_flag = 1;
					sending_flag = 0;
					link_number = -1;
					connection_counter = 0;
					timer_count = 0;
					HAL_UART_Transmit(&huart1, (uint8_t*) uart2_rx_buffer,
							data_length, 0xffff);
				}
			} else if (strncmp(uart2_rx_buffer, "0,CONNECT\r\n", 11) == 0) {
				//server 端要设置
				link_number = 0;
				connect_flag = 2;
				//StateChange(connect_flag);
				HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET); //0
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); //1
				HAL_UART_Transmit(&huart1, (uint8_t*) "Connection on id 0\r\n",
						20, 0xffff);
			} else if (strncmp(uart2_rx_buffer, "0,CONNECT FAIL\r\n", 16) == 0||
			strstr(uart2_rx_buffer, "0,CLOSED\r\n") != NULL) {
				init_var();
				HAL_UART_Transmit(&huart1,
						(uint8_t*) "Close connection on id 0\r\n", 26, 0xffff);
			} else if (strncmp(uart2_rx_buffer, "0,CLOSED\r\n", 10) == 0) {
				connect_flag = 1;
				sending_flag = 0;
				link_number = -1;
				connection_counter = 0;
				timer_count = 0;
				HAL_UART_Transmit(&huart1,
						(uint8_t*) "connection closed on id 0\r\n", 26, 0xffff);
			} else {
//				uint8_t msg[15];
//				sprintf(msg, "data length=%d\r\n", data_length);
//				HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), 0xffff);
				// send to uart1 again, to show on the usart1 display

				HAL_UART_Transmit(&huart1, (uint8_t*) uart2_rx_buffer,
						data_length, 0xffff);
			}
			memset(uart2_rx_buffer, 0, data_length);
			memset(received_Data,0,2048);
			data_length = 0;
			HAL_UART_Receive_DMA(&huart2, (uint8_t*) uart2_rx_buffer, 2048);
		}
	}
}

void send_cmd(uint8_t *cmd, int delay_time) {
	HAL_UART_Transmit(&huart2, (uint8_t*) cmd, strlen(cmd), 0xffff);
	HAL_Delay(delay_time);
}

void send_message(uint8_t *msg) {
	HAL_Delay(400);
	uint8_t activate[100];
	sprintf(activate, "AT+CIPSEND=%d,%d\r\n", link_number, strlen(msg));
	send_cmd((uint8_t*) activate, 2000);
	send_msg_uart1(msg, 500);
	//printOut(msg, strlen(msg), 1, connect_flag);
	send_cmd(msg, 1000);
}

void send_msg_uart1(uint8_t *msg, int delay_time) {
	HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), 0xffff);
	HAL_Delay(delay_time);
}

void init_var() {
	connect_flag = 0;
	sending_flag = 0;
	link_number = -1;
	connection_counter = 0;
	timer_count = 0;
	//StateChange(connect_flag);
	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
}

void connect_to_wifi() {
	init_var();
	send_msg_uart1((uint8_t*) "start connect to wifi\r\n", 2000);
	send_cmd("AT+RST\r\n", 5000);
	send_cmd("AT+CWMODE=1\r\n", 2000);
	send_cmd("AT+RST\r\n", 5000);
	send_cmd("AT+CWJAP=\"SUSTC-WIFI-FAKE\",\"987654321\"\r\n", 10000);
	send_cmd("AT+CIPMUX=1\r\n", 5000);
	send_cmd("AT+CIPSERVER=1\r\n", 5000);
}

void init_server() {
	init_var();
	send_msg_uart1((uint8_t*) "start initialize server\r\n", 0);
	send_cmd("AT+CWMODE=3\r\n", 2000);
	send_cmd("AT+RST\r\n", 5000);
	send_cmd("AT+CWSAP=\"SUSTC-WIFI-FAKE\",\"987654321\",1,0,4,0\r\n", 2000);
	send_cmd("AT+CIPMUX=1\r\n", 2000);
	send_cmd("AT+CIPSERVER=1,8089\r\n", 2000);
	send_msg_uart1((uint8_t*) "end initialize server\r\n", 0);
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
