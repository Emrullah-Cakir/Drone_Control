	/* USER CODE BEGIN Header */
		/**
			******************************************************************************
			* @file           : main.c
			* @brief          : Main program body
			******************************************************************************
			* @attention
			*
			* Copyright (c) 2025 STMicroelectronics.
			* All rights reserved.
			*
			* This software is licensed under terms that can be found in the LICENSE file
			* in the root directory of this software component.
			* If no LICENSE file comes with this software, it is provided AS-IS.
			*
			******************************************************************************
			*/
	/* USER CODE END Header */
	/* Includes ------------------------------------------------------------------*/
	#include "main.h"
	#include "i2c.h"
	#include "usart.h"
	#include "gpio.h"

	/* Private includes ----------------------------------------------------------*/
	/* USER CODE BEGIN Includes */
		#include "adxl345.h"
		#include "all/mavlink.h"
		#include "common/mavlink_msg_manual_control.h"
	/* USER CODE END Includes */

	/* Private typedef -----------------------------------------------------------*/
	/* USER CODE BEGIN PTD */
		typedef enum {
			STATE_IDLE,             // Baslangi� beklemesi
			STATE_SET_STABILIZE,    // Stabilize mod komutu g�nder
			STATE_WAIT_STABILIZE,   // Modun degistigini teyit et
			STATE_SEND_ARM,         // Arm komutu g�nder
			STATE_WAIT_ARM,         // Arm oldugunu teyit et
			STATE_FLYING            // U�us ve ALT_HOLD modu
		} DroneState;
	/* USER CODE END PTD */

	/* Private define ------------------------------------------------------------*/
	/* USER CODE BEGIN PD */
		#define RX_BUFFER_SIZE 256
	/* USER CODE END PD */

	/* Private macro -------------------------------------------------------------*/
	/* USER CODE BEGIN PM */

	/* USER CODE END PM */

	/* Private variables ---------------------------------------------------------*/

	/* USER CODE BEGIN PV */
		int16_t x_val = 0;
		int16_t y_val = 0;
		int16_t z_val = 0;
		uint32_t last_heartbeat_time = 0;
		int16_t is_armed = 0;
		DroneState current_state = STATE_IDLE;
		uint8_t remote_armed = 0;    // Otopilottan gelen ger�ek Arm durumu
		uint32_t remote_mode = 999;  // Otopilottan gelen ger�ek Mod bilgisi
		int16_t current_throttle = 0;
		uint32_t last_retry_time = 0;
		uint8_t rx_buffer[RX_BUFFER_SIZE]; // Gelen ham veriler buraya akar
		uint8_t temp_byte;                 // Her seferinde 1 byte almak i�in
		#define RING_BUFFER_SIZE 1024
		uint8_t ring_buffer[RING_BUFFER_SIZE];
		volatile uint16_t head = 0; // Yazma indisi (Interrupt kullanacak)
		volatile uint16_t tail = 0; // Okuma indisi (Main d�ng�s� kullanacak)
		uint8_t rx_tmp;
		int16_t in_min = -145; 
		int16_t in_max = 145;
		int16_t out_min = -1000;
		int16_t out_max = 1000;
		static uint8_t mav_tx_buf[MAVLINK_MAX_PACKET_LEN];
	/* USER CODE END PV */

	/* Private function prototypes -----------------------------------------------*/
	void SystemClock_Config(void);
	/* USER CODE BEGIN PFP */
		uint8_t available_in_buffer(void);
		uint8_t pop_from_buffer(void);
		void push_to_buffer(uint8_t data);
		int16_t map_value(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max);
		void MAVLink_Send_Heartbeat(void);
		void MAVLink_Set_Mode_Copter(uint32_t copter_mode);
		void MAVLink_Arm_Copter(uint8_t arm_status);
		void MAVLink_Send_ManualControl(int16_t pitch, int16_t roll, int16_t throttle);
	/* USER CODE END PFP */

	/* Private user code ---------------------------------------------------------*/
	/* USER CODE BEGIN 0 */

	/* USER CODE END 0 */

	/**
		* @brief  The application entry point.
		* @retval int
		*/
	int main(void)
	{

		/* USER CODE BEGIN 1 */
			uint8_t rx_byte;
			mavlink_message_t rx_msg;
			mavlink_status_t rx_status;
			uint8_t byte;
			int16_t x, y;
			int16_t pitch_cmd, roll_cmd;
		/* USER CODE END 1 */

		/* MCU Configuration--------------------------------------------------------*/

		/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
		HAL_Init();

		/* USER CODE BEGIN Init */

		/* USER CODE END Init */

		/* Configure the system clock */
		SystemClock_Config();

		/* USER CODE BEGIN SysInit */

		/* USER CODE END SysInit */

		/* Initialize all configured peripherals */
		MX_GPIO_Init();
		MX_I2C1_Init();
		MX_USART2_UART_Init();
		/* USER CODE BEGIN 2 */
			adxl_init();
			HAL_UART_Receive_IT(&huart2, &rx_tmp, 1);
		/* USER CODE END 2 */

		/* Infinite loop */
		/* USER CODE BEGIN WHILE */
			while (1)
			{
			/* USER CODE END WHILE */

			/* USER CODE BEGIN 3 */

		// --- 1. OTOPILOTU DINLE (FEEDBACK) ---
			
			
			// UART �zerinden otopilottan gelen verileri oku (Non-blocking)
			while (available_in_buffer()) {
					uint8_t byte = pop_from_buffer();
			
					// MAVLink paketini ��z
					if (mavlink_parse_char(MAVLINK_COMM_0, byte, &rx_msg, &rx_status)) {
							if (rx_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
									mavlink_heartbeat_t hb;
									mavlink_msg_heartbeat_decode(&rx_msg, &hb);
									remote_mode = hb.custom_mode;
									remote_armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) ? 1 : 0;
							
						}
						else if (rx_msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
							mavlink_statustext_t status;
							mavlink_msg_statustext_decode(&rx_msg, &status);
						}
					}
			} 
	
			
			if (HAL_GetTick() - last_heartbeat_time >= 1000) {
				MAVLink_Send_Heartbeat();
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
				last_heartbeat_time = HAL_GetTick();
			}
			// --- 2. DURUM MAKINESI (LOGIC) ---
			switch (current_state) {
				case STATE_IDLE:
					if (HAL_GetTick() > 5000) current_state = STATE_SET_STABILIZE;
					break;

				case STATE_SET_STABILIZE:
					MAVLink_Set_Mode_Copter(0); // 0 = STABILIZE
					last_retry_time = HAL_GetTick();
					current_state = STATE_WAIT_STABILIZE;
					break;

				case STATE_WAIT_STABILIZE:
					if (remote_mode == 0) {
						current_state = STATE_SEND_ARM;
					} else if (HAL_GetTick() - last_retry_time > 2000) {
						current_state = STATE_SET_STABILIZE; // 2 sn sonra tekrar dene
					}
					break;

				case STATE_SEND_ARM:
					MAVLink_Arm_Copter(1); // 1 = ARM
					last_retry_time = HAL_GetTick();
					current_state = STATE_WAIT_ARM;
					break;

				case STATE_WAIT_ARM:
					if (remote_armed == 1) {
						current_state = STATE_FLYING;
						// Arm olduktan sonra yerden kesilmek i�in gazi veriyoruz
						current_throttle = 600; 
					} else if (HAL_GetTick() - last_retry_time > 1000) {
						current_state = STATE_SEND_ARM; // 1 sn sonra tekrar dene
					}
					break;

				case STATE_FLYING:{
					// Irtifa kazandiktan sonra (�rnegin 2 sn sonra) ALT_HOLD'a ge�is
					static uint32_t flying_start = 0;
					if (flying_start == 0) flying_start = HAL_GetTick();
					
					if (HAL_GetTick() - flying_start > 2000 && remote_mode != 2) {
						MAVLink_Set_Mode_Copter(2); // 2 = ALT_HOLD
						current_throttle = 500;    // ALT_HOLD'da 500 asili kalma degeridir
					}
					break;
			}

			// --- 3. PERIYODIK G�NDERIMLER ---

			// Heartbeat (1Hz)
			

			// Manual Control (20Hz - Her 50ms'de bir)
			static uint32_t last_ctrl_time = 0;
			if (HAL_GetTick() - last_ctrl_time >= 50) {
				 x = adxl_readx();
				 y = adxl_ready();

				if (x > -3 && x < 3) x = 0;
					if (y > -3 && y < 3) y = 0;

				int16_t pitch_cmd = map_value(x, -145, 145, -1000, 1000);
				int16_t roll_cmd  = map_value(y, -145, 145, -1000, 1000);

				
				// MANUAL_CONTROL mesajini paketle ve g�nder
				MAVLink_Send_ManualControl(pitch_cmd, roll_cmd, current_throttle);
				last_ctrl_time = HAL_GetTick();
			}		
			}
		}
		/* USER CODE END 3 */
	}

	/**
		* @brief System Clock Configuration
		* @retval None
		*/
	void SystemClock_Config(void)
	{
		RCC_OscInitTypeDef RCC_OscInitStruct = {0};
		RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

		/** Configure the main internal regulator output voltage
		*/
		__HAL_RCC_PWR_CLK_ENABLE();
		__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

		/** Initializes the RCC Oscillators according to the specified parameters
		* in the RCC_OscInitTypeDef structure.
		*/
		RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
		RCC_OscInitStruct.HSIState = RCC_HSI_ON;
		RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
		RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
		RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
		RCC_OscInitStruct.PLL.PLLM = 16;
		RCC_OscInitStruct.PLL.PLLN = 336;
		RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
		RCC_OscInitStruct.PLL.PLLQ = 7;
		if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
		{
			Error_Handler();
		}

		/** Initializes the CPU, AHB and APB buses clocks
		*/
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
	}

	/* USER CODE BEGIN 4 */
		int16_t map_value(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max) {
			// 1. SINIRLAMA (Clamping): Deger giris araliginin disina �ikmasin
			if (x < in_min) x = in_min;
			if (x > in_max) x = in_max;

			// 2. TIP D�N�S�M� (Casting): �arpma islemini 32-bit yaparak tasmanin �n�ne ge�iyoruz
			// int32_t 2 milyara kadar olan sayilari tasiyabilir (470.000 artik sorun degil)
			int32_t result = (int32_t)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

			return (int16_t)result;
	}

		void MAVLink_Send_Heartbeat(void) {
				mavlink_message_t msg;
				uint8_t buf[MAVLINK_MAX_PACKET_LEN];
				mavlink_msg_heartbeat_pack(100, 1, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);

				uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
				HAL_UART_Transmit(&huart2, buf, len, HAL_MAX_DELAY);
		}
		// ArduCopter Mod Degistirme: 0=STABILIZE, 4=GUIDED, 5=LOITER
		void MAVLink_Set_Mode_Copter(uint32_t copter_mode) {
				mavlink_message_t msg;
				uint8_t buf[MAVLINK_MAX_PACKET_LEN];

				mavlink_msg_command_long_pack(
						100, 1, &msg, 
						1, 1,                          // Hedef Sistem ID, Hedef Bilesen ID
						MAV_CMD_DO_SET_MODE,           // Komut ID
						0,                             // Confirmation
						MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, // ArduPilot i?in bu 1 olmali
						(float)copter_mode,            // Mod numarasi
						0, 0, 0, 0, 0                  // Diger parametreler bos
				);

				uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
				HAL_UART_Transmit(&huart2, buf, len, HAL_MAX_DELAY);
		}

		// Arm/Disarm: 1=ARM, 0=DISARM
		void MAVLink_Arm_Copter(uint8_t arm_status) {
				mavlink_message_t msg;
				uint8_t buf[MAVLINK_MAX_PACKET_LEN];

				mavlink_msg_command_long_pack(
						100, 1, &msg, 
						1, 1, 
						MAV_CMD_COMPONENT_ARM_DISARM, 
						0, 
						(float)arm_status,             // 1.0f = ARM
						0, 0, 0, 0, 0, 0
				);

				uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
				HAL_UART_Transmit(&huart2, buf, len, HAL_MAX_DELAY);
		}

		void MAVLink_Send_ManualControl(int16_t pitch, int16_t roll, int16_t throttle) {
			mavlink_message_t msg;
			uint8_t buf[MAVLINK_MAX_PACKET_LEN];
			mavlink_msg_manual_control_pack(100, 1, &msg, 1, pitch, roll, throttle, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
			HAL_UART_Transmit(&huart2, buf, len, 100);
		}



		uint8_t available_in_buffer(void) {
			return (head != tail);
		}

		uint8_t pop_from_buffer(void) {
			if (head == tail) return 0;
			uint8_t data = ring_buffer[tail];
			tail = (tail + 1) % RING_BUFFER_SIZE;
			return data;
		}

		void push_to_buffer(uint8_t data) {
			uint16_t next_head = (head + 1) % RING_BUFFER_SIZE;
			if (next_head != tail) { // Buffer dolu degilse yaz (Tasma korumasi)
					ring_buffer[head] = data;
					head = next_head;}
		}

		void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
			if (huart->Instance == USART2) {
					push_to_buffer(rx_tmp); // Gelen byte'i tampona at
					HAL_UART_Receive_IT(&huart2, &rx_tmp, 1); // Bir sonraki byte i�in kesmeyi tekrar a�
			}
		}
	/* USER CODE END 4 */

	/**
		* @brief  This function is executed in case of error occurrence.
		* @retval None
		*/
	void Error_Handler(void)
	{
		/* USER CODE BEGIN Error_Handler_Debug */
			/* User can add his own implementation to report the HAL error return state */
			__disable_irq();
			while (1)
			{
			}
		/* USER CODE END Error_Handler_Debug */
	}
	#ifdef USE_FULL_ASSERT
	/**
		* @brief  Reports the name of the source file and the source line number
		*         where the assert_param error has occurred.
		* @param  file: pointer to the source file name
		* @param  line: assert_param error line source number
		* @retval None
		*/
	void assert_failed(uint8_t *file, uint32_t line)
	{
		/* USER CODE BEGIN 6 */
			/* User can add his own implementation to report the file name and line number,
				 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
		/* USER CODE END 6 */
	}
	#endif /* USE_FULL_ASSERT */
