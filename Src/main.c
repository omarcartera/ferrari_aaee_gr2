// what file is this in Priya's code?
#include "main.h"

#define KEY_PRESSED     0x00
#define KEY_NOT_PRESSED 0x01

CAN_HandleTypeDef CanHandle;
SMBUS_HandleTypeDef hsmbus1;
I2C_HandleTypeDef hi2c2;
SPI_HandleTypeDef hspi2;
TSC_HandleTypeDef htsc;

uint8_t ubKeyNumber = 0x0;
unsigned char buffer[5];
unsigned char bufferi2[5];
unsigned char bufferi2_2[5];
unsigned char bufferi2_3[5];
unsigned char bufferi2_4[5];
unsigned char buffer_out_acc[8];
unsigned char buffer_out_pre[8];
unsigned char buffer_read[3];
unsigned int rawX, rawY, rawZ;
float pressure;
float pressure_LSB;

void SystemClock_Config(void);
static void Error_Handler(void);
static void CAN_Config(void);
static void LED_Display(uint8_t LedStatus);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_SMBUS_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TSC_Init(void);

int main(void)
{
	HAL_Init();

	/* Configure the system clock to 48 MHz */
	SystemClock_Config();

	/* Configure LED1, LED2, LED3 and LED4 */
	BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);
	BSP_LED_Init(LED3);
	BSP_LED_Init(LED4);

	/* Configure Tamper push-button */
	BSP_PB_Init(BUTTON_TAMPER, BUTTON_MODE_GPIO);

	/* Configure the CAN peripheral */
	MX_GPIO_Init();
	MX_CAN_Init();
	MX_I2C1_SMBUS_Init();
	MX_I2C2_Init();
	MX_SPI2_Init();
	MX_TSC_Init();
	CAN_Config();

	buffer[0] = 0X0B;		/*pointer*/
	buffer[1] = 0;			/*MSB*/
	/*buffer[2] = 0;    	/*LSB*/

	bufferi2[0] = 0X13; 	/*pointer*/
	bufferi2[1] = 0;    	/*MSB*/
	/*bufferi2[2] = 0;    	/*LSB*/

	bufferi2_2[0] = 0X0F; 	/*pointer for range*/
	bufferi2_2[1] = C;    	/*MSB*/
	/*bufferi2_2[2] = 0;    /*LSB*/


	bufferi2_3[0] = 0X10; 	/*pointer for bandwidth*/
	bufferi2_3[1] = 0X09;	/*MSB*/
	/* bufferi2_3[2] = 0;	/*LSB*/


	bufferi2_4[0] = 0X3E;	/*pointer for fifo configuration stream  mode*/
	bufferi2_4[1] = 0X80;	/*MSB*/
	/* bufferi2_3[2] = 0;	/*LSB*/

	HAL_I2C_Master_Transmit(&hi2c1, 0X5D<<1, buffer, 2, 100);
	HAL_I2C_Master_Transmit(&hi2c1, 0X19<<1, bufferi2, 2, 100); 		/* Setting for filtered data*/
	HAL_I2C_Master_Transmit(&hi2c1, 0X19<<1, bufferi2_2, 2, 100); 	/* Setting for bandwidth*/
	HAL_I2C_Master_Transmit(&hi2c1, 0X19<<1, bufferi2_3, 2, 100); 	/* Setting for range*/
	HAL_I2C_Master_Transmit(&hi2c1, 0X19<<1, bufferi2_4, 2, 100); 	/* Setting for fifo configuration stream mode*/

	/* Start the Reception process and enable reception interrupt */
	if (HAL_OK != HAL_CAN_Receive_IT(&CanHandle, CAN_FIFO0))
	{
		Error_Handler();
	}

	/* Infinite loop */
	while (1)
	{
		buffer_read[0] = 0X28;
		HAL_I2C_Master_Transmit(&hi2c1, 0X5D<<1, buffer_read[0], 1, 100);
		HAL_Delay(20);
		HAL_I2C_Master_Receive(&hi2c1, 0X5D<<1, buffer_out_pre[0], 1, 100);

		buffer_read[1] = 0X29;
		HAL_I2C_Master_Transmit(&hi2c1, 0X5D<<1, buffer_read[1], 1, 100);
		HAL_Delay(20);
		HAL_I2C_Master_Receive(&hi2c1, 0X5D<<1, buffer_out_pre[1], 1, 100);

		buffer_read[2] = 0X2A;
		HAL_I2C_Master_Transmit(&hi2c1, 0X5D<<1, buffer_read[2], 1, 100);
		HAL_Delay(20);
		HAL_I2C_Master_Receive(&hi2c1, 0X5D<<1, buffer_out_pre[2], 1, 100);


		pressure_LSB = buffer_out_pre[0] & buffer_out_pre[1] & buffer_out_pre[2];
		pressure = pressure_LSB / 4096; /*pressure in hpa*/


		buffer_read[0] = 0X3F;
		HAL_I2C_Master_Transmit(&hi2c1, 0X19<<1, buffer_read[0], 1, 100);
		HAL_Delay(20);
		HAL_I2C_Master_Receive(&hi2c1, 0X19<<1, buffer_out_acc, 6, 100);

		rawX = buffer_out_acc[0] << 8 | buffer_out_acc[1];	//combine 2 8-bit into 1 16bit, X axis
		rawY = buffer_out_acc[2] << 8 | buffer_out_acc[3];	//combine 2 8-bit into 1 16bit, Y axis
		rawZ = buffer_out_acc[4] << 8 | buffer_out_acc[5];	//combine 2 8-bit into 1 16bit ,Z axis

		while (KEY_PRESSED == BSP_PB_GetState(BUTTON_TAMPER))
		{
			if (0x04 == ubKeyNumber)
			{
				ubKeyNumber = 0x00;
			}

			else
			{
				LED_Display(++ubKeyNumber);

				/* Set the data to be transmitted, the others are zero */
				CanHandle.pTxMsg->Data[0] = buffer_out_acc[0];
				CanHandle.pTxMsg->Data[1] = buffer_out_acc[1];
				CanHandle.pTxMsg->Data[2] = buffer_out_acc[2];
				CanHandle.pTxMsg->Data[3] = buffer_out_acc[3];
				CanHandle.pTxMsg->Data[4] = buffer_out_acc[4];
				CanHandle.pTxMsg->Data[5] = buffer_out_acc[5];
				CanHandle.pTxMsg->Data[6] = (char) pressure_LSB;

				/* Start the Transmission process */
				if (HAL_OK != HAL_CAN_Transmit(&CanHandle, 10))
				{
					/* Transmission Error */
					Error_Handler();
				}

				HAL_Delay(10);

				while (KEY_NOT_PRESSED != BSP_PB_GetState(BUTTON_TAMPER));
			}
		}
	}
}

void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* Enable HSE Oscillator and Activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType 	= RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState 			= RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState 		= RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource 	= RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PREDIV 		= RCC_PREDIV_DIV1;
	RCC_OscInitStruct.PLL.PLLMUL 		= RCC_PLL_MUL6;

	if (HAL_OK != HAL_RCC_OscConfig(&RCC_OscInitStruct))
	{
		Error_Handler();
	}

	/* Select PLL as system clock source and configure the HCLK, PCLK1 clocks dividers */
	RCC_ClkInitStruct.ClockType 		= (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
	RCC_ClkInitStruct.SYSCLKSource 		= RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider 	= RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider 	= RCC_HCLK_DIV1;

	if (HAL_OK != HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1))
	{
		Error_Handler();
	}
}

static void Error_Handler(void)
{
	while (1);
}

static void CAN_Config(void)
{
	CAN_FilterConfTypeDef		sFilterConfig;
	static CanTxMsgTypeDef		TxMessage;
	static CanRxMsgTypeDef		RxMessage;

	/*Configure the CAN peripheral */
	CanHandle.Instance 			= CANx;
	CanHandle.pTxMsg 			= &TxMessage;
	CanHandle.pRxMsg 			= &RxMessage;

	CanHandle.Init.TTCM 		= DISABLE;
	CanHandle.Init.ABOM 		= DISABLE;
	CanHandle.Init.AWUM 		= DISABLE;
	CanHandle.Init.NART 		= DISABLE;
	CanHandle.Init.RFLM 		= DISABLE;
	CanHandle.Init.TXFP 		= DISABLE;
	CanHandle.Init.Mode 		= CAN_MODE_NORMAL;
	CanHandle.Init.SJW  		= CAN_SJW_1TQ;
	CanHandle.Init.BS1  		= CAN_BS1_5TQ;
	CanHandle.Init.BS2  		= CAN_BS2_6TQ;
	CanHandle.Init.Prescaler 	= 4;

	if (HAL_OK != HAL_CAN_Init(&CanHandle))
	{
		Error_Handler();
	}

	/* Configure the CAN Filter */
	sFilterConfig.FilterNumber 			= 0;
	sFilterConfig.FilterMode 			= CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale 			= CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh 			= 0x0000;
	sFilterConfig.FilterIdLow 			= 0x0000;
	sFilterConfig.FilterMaskIdHigh 		= 0x0000;
	sFilterConfig.FilterMaskIdLow 		= 0x0000;
	sFilterConfig.FilterFIFOAssignment 	= 0;
	sFilterConfig.FilterActivation 		= ENABLE;
	sFilterConfig.BankNumber 			= 14;

	if (HAL_OK != HAL_CAN_ConfigFilter(&CanHandle, &sFilterConfig))
	{
		Error_Handler();
	}

	/* Configure Transmission process */
	CanHandle.pTxMsg->StdId 	= 0x321;
	CanHandle.pTxMsg->ExtId 	= 0x01;
	CanHandle.pTxMsg->RTR 		= CAN_RTR_DATA;
	CanHandle.pTxMsg->IDE 		= CAN_ID_STD;
	CanHandle.pTxMsg->DLC 		= 7;	// must be generalised
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *CanHandle)
{
	if ((0x321 == CanHandle->pRxMsg->StdId) && (CAN_ID_STD == CanHandle->pRxMsg->IDE) && (2 == CanHandle->pRxMsg->DLC))
	{
		LED_Display(CanHandle->pRxMsg->Data[0]);
		ubKeyNumber = CanHandle->pRxMsg->Data[0];
	}

	/* Receive */
	if (HAL_OK != HAL_CAN_Receive_IT(CanHandle, CAN_FIFO0))
	{
		Error_Handler();
	}
}

void LED_Display(uint8_t LedStatus)
{
	/* Turn OFF all LEDs */
	BSP_LED_Off(LED1);
	BSP_LED_Off(LED2);
	BSP_LED_Off(LED3);
	BSP_LED_Off(LED4);

	switch (LedStatus)
	{
	case (1):
		BSP_LED_On(LED1);
		break;

	case (2):
		BSP_LED_On(LED2);
		break;

	case (3):
		BSP_LED_On(LED3);
		break;

	case (4):
		BSP_LED_On(LED4);
		break;
		
	default:
		break;
	}
}

static void MX_I2C1_SMBUS_Init(void)
{
	hsmbus1.Instance 					= I2C1;
	hsmbus1.Init.Timing 				= 0x2000090E;
	hsmbus1.Init.AnalogFilter 			= SMBUS_ANALOGFILTER_ENABLE;
	hsmbus1.Init.OwnAddress1 			= 2;
	hsmbus1.Init.AddressingMode 		= SMBUS_ADDRESSINGMODE_7BIT;
	hsmbus1.Init.DualAddressMode		= SMBUS_DUALADDRESS_DISABLE;
	hsmbus1.Init.OwnAddress2			= 0;
	hsmbus1.Init.OwnAddress2Masks		= SMBUS_OA2_NOMASK;
	hsmbus1.Init.GeneralCallMode		= SMBUS_GENERALCALL_DISABLE;
	hsmbus1.Init.NoStretchMode			= SMBUS_NOSTRETCH_DISABLE;
	hsmbus1.Init.PacketErrorCheckMode	= SMBUS_PEC_DISABLE;
	hsmbus1.Init.PeripheralMode			= SMBUS_PERIPHERAL_MODE_SMBUS_SLAVE;
	hsmbus1.Init.SMBusTimeout			= 0x00008061;

	if (HAL_SMBUS_Init(&hsmbus1) != HAL_OK)
	{
		Error_Handler();
	}

	/* configuration Alert Mode */
	if (HAL_SMBUS_EnableAlert_IT(&hsmbus1) != HAL_OK)
	{
		Error_Handler();
	}
}


static void MX_I2C2_Init(void)
{
	hi2c2.Instance 					= I2C2;
	hi2c2.Init.Timing 				= 0x20303E5D;
	hi2c2.Init.OwnAddress1 			= 0;
	hi2c2.Init.AddressingMode 		= I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode 		= I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 			= 0;
	hi2c2.Init.OwnAddress2Masks		= I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode 		= I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode 		= I2C_NOSTRETCH_DISABLE;

	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}

	/* Configure Analogue filter */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/* Configure Digital filter */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_SPI2_Init(void)
{
	/* SPI2 parameter configuration*/
	hspi2.Instance 					= SPI2;
	hspi2.Init.Mode 				= SPI_MODE_MASTER;
	hspi2.Init.Direction 			= SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize 			= SPI_DATASIZE_4BIT;
	hspi2.Init.CLKPolarity 			= SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase 			= SPI_PHASE_1EDGE;
	hspi2.Init.NSS 					= SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler 	= SPI_BAUDRATEPRESCALER_4;
	hspi2.Init.FirstBit 			= SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode 				= SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation 		= SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial 		= 7;
	hspi2.Init.CRCLength 			= SPI_CRC_LENGTH_DATASIZE;
	hspi2.Init.NSSPMode 			= SPI_NSS_PULSE_ENABLE;

	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_TSC_Init(void)
{
	/* Configure the TSC peripheral */
	htsc.Instance 						= TSC;
	htsc.Init.CTPulseHighLength 		= TSC_CTPH_2CYCLES;
	htsc.Init.CTPulseLowLength 			= TSC_CTPL_2CYCLES;
	htsc.Init.SpreadSpectrum 			= DISABLE;
	htsc.Init.SpreadSpectrumDeviation 	= 1;
	htsc.Init.SpreadSpectrumPrescaler 	= TSC_SS_PRESC_DIV1;
	htsc.Init.PulseGeneratorPrescaler 	= TSC_PG_PRESC_DIV4;
	htsc.Init.MaxCountValue 			= TSC_MCV_8191;
	htsc.Init.IODefaultMode 			= TSC_IODEF_OUT_PP_LOW;
	htsc.Init.SynchroPinPolarity 		= TSC_SYNC_POLARITY_FALLING;
	htsc.Init.AcquisitionMode 			= TSC_ACQ_MODE_NORMAL;
	htsc.Init.MaxCountInterrupt 		= DISABLE;
	htsc.Init.ChannelIOs 				= TSC_GROUP1_IO3|TSC_GROUP2_IO3|TSC_GROUP3_IO2;
	htsc.Init.ShieldIOs 				= 0;
	htsc.Init.SamplingIOs 				= TSC_GROUP1_IO4|TSC_GROUP2_IO4|TSC_GROUP3_IO3;

	if (HAL_TSC_Init(&htsc) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC,
					  NCS_MEMS_SPI_Pin | EXT_RESET_Pin|LD3_Pin | LD6_Pin | LD4_Pin | LD5_Pin,
					  GPIO_PIN_RESET);

	/* Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

	/* Configure GPIO pins: NCS_MEMS_SPI_Pin EXT_RESET_Pin LD3_Pin LD6_Pin LD4_Pin LD5_Pin */
	GPIO_InitStruct.Pin 	= NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin |LD4_Pin|LD5_Pin;
	GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull 	= GPIO_NOPULL;
	GPIO_InitStruct.Speed 	= GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* Configure GPIO pins : MEMS_INT1_Pin MEMS_INT2_Pin */
	GPIO_InitStruct.Pin 	= MEMS_INT1_Pin|MEMS_INT2_Pin;
	GPIO_InitStruct.Mode 	= GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull 	= GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin 	= B1_Pin;
	GPIO_InitStruct.Mode 	= GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull 	= GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/* Configure GPIO pin : PB8 */
	GPIO_InitStruct.Pin 	= GPIO_PIN_8;
	GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull 	= GPIO_NOPULL;
	GPIO_InitStruct.Speed 	= GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
