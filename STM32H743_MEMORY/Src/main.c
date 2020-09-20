/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/*********************	SDRAM	*********************/
#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)
#define SDRAM_REFRESH_COUNT                  	 ((uint32_t)0x050C)
#define SDRAM_TIMEOUT                   		 ((uint32_t)0xFFFF)

#if defined ( __ICCARM__ ) // !< IAR Compiler
#pragma location=0xC0000000
static __no_init uint32_t sdram_data1[100];
#pragma location=0xD0000000
static __no_init uint32_t sdram_data2[100];
#elif defined ( __CC_ARM ) //!< Keil Compiler
uint32_t sdram_data1[100] __attribute__((at(0xC0000000)));
uint32_t sdram_data2[100] __attribute__((at(0xD0000000)));

#elif defined ( __GNUC__ ) // !< GNU Compiler
uint32_t sdram_data1[100]__attribute__((section(".sdram1")));//Must add sdram1 script to STM32H743ZITx_FLASH.ld
uint32_t sdram_data2[100]__attribute__((section(".sdram2")));//Must add sdram2 script to STM32H743ZITx_FLASH.ld
#endif

/*********************	SRAM	*********************/
#define SRAM_BANK_ADDR1  						 ((uint32_t)0x60000000)
#define SRAM_BANK_ADDR2  						 ((uint32_t)0x68000000)
#define SRAM_MEMORY_WIDTH          				 FMC_NORSRAM_MEM_BUS_WIDTH_16
#define SRAM_BUFFER_SIZE        				 ((uint32_t)0x1000)
#define SRAM_WRITE_READ_ADDR     				 ((uint32_t)0x0000)

uint32_t sram_aTxBuffer[SRAM_BUFFER_SIZE];
uint32_t sram_aRxBuffer[SRAM_BUFFER_SIZE];
__IO uint32_t sram_uwWriteReadStatus = 0;
uint32_t sram_uwIndex = 0;

/*****************	NOR FLASH	*******************/
#define NOR_BANK_ADDR                 			 ((uint32_t)0x6C000000)
#define NOR_PROGRAM_TIMEOUT             		 ((uint32_t)0x00004400)
#define NOR_ERASE_TIMEOUT               		 ((uint32_t)0x00A00000)
#define NOR_MEMORY_WIDTH              			 FMC_NORSRAM_MEM_BUS_WIDTH_16
#define NOR_CONTINUOUS_CLOCK					 FMC_CONTINUOUS_CLOCK_SYNC_ASYNC
#define NOR_BUFFER_SIZE        					 ((uint32_t)0x1000)
#define NOR_WRITE_READ_ADDR     				 ((uint32_t)0x0800)

uint16_t nor_aTxBuffer[NOR_BUFFER_SIZE];
uint16_t nor_aRxBuffer[NOR_BUFFER_SIZE];
__IO uint32_t nor_uwWriteReadStatus = 0;
uint32_t nor_uwIndex = 0;


/*****************	NAND FLASH	*******************/
//#define NAND_BANK_ADDR   						((uint32_t)0x80000000)
#define NAND_TIMEOUT     						((uint32_t)0xFFFF)
#define WRITE_READ_ADDR         				((uint32_t)0x8000)
#define NAND_MAKERID         					((uint32_t)0x2C)
#define NAND_DEVICEID        					((uint32_t)0xD3)
#define NAND_PAGE_SIZE          				((uint16_t)0x1000)
#define NAND_BLOCK_SIZE        					((uint16_t)0x0040)
#define NAND_BLOCK_NBR        					((uint16_t)0x0800)
#define NAND_PLANE_SIZE        					((uint16_t)0x0800)
#define NAND_PLANE_NBR        					((uint16_t)0x0002)
#define NAND_SPARE_AREA_SIZE    				((uint16_t)0x0010)
#define NAND_NB_PAGE                 			((uint32_t)1)
#define NAND_BUFFER_SIZE             			(NAND_PAGE_SIZE * NAND_NB_PAGE)
//#define ECC_ENABLE
static NAND_IDTypeDef NAND_Id;
static NAND_AddressTypeDef NAND_Address;
uint8_t nand_aTxBuffer[NAND_BUFFER_SIZE];
uint8_t nand_aRxBuffer[NAND_BUFFER_SIZE];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */ 

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

ETH_TxPacketConfig TxConfig; 
ETH_HandleTypeDef heth;

UART_HandleTypeDef huart3;
SRAM_HandleTypeDef hsram1;
SRAM_HandleTypeDef hsram2;
NOR_HandleTypeDef hnor3;
NAND_HandleTypeDef hnand1;
SDRAM_HandleTypeDef hsdram1;
SDRAM_HandleTypeDef hsdram2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_FMC_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

static void NAND_Fill_Buffer(uint8_t *pBuffer, uint32_t uwBufferLenght, uint32_t uwOffset);
static TestStatus NAND_Buffercmp(uint8_t* pBuffer, uint8_t* pBuffer1, uint32_t uwBufferLenght);
static void SRAM_Fill_Buffer(uint32_t *pBuffer, uint32_t uwBufferLength, uint16_t uwOffset);
static uint8_t SRAM_Buffercmp(uint32_t* pBuffer1, uint32_t* pBuffer2, uint16_t BufferLength);
static void NOR_Fill_Buffer(uint16_t *pBuffer, uint32_t uwBufferLenght, uint16_t uwOffset);
static TestStatus NOR_Buffercmp(uint16_t* pBuffer1, uint16_t* pBuffer2, uint16_t BufferLength);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write( int file, char *ptr, int len )
{
   HAL_UART_Transmit(&huart3,(uint8_t*)ptr, len, 1000);
  return len;
}

/*****************	SDRAM *******************/
void SDRAM_Initialization_sequence1(SDRAM_HandleTypeDef  *sdramHandle, uint32_t RefreshCount)
{

	__IO uint32_t tmpmrd = 0;
	FMC_SDRAM_CommandTypeDef Command;
	/* Configure a clock configuration enable command */
	Command.CommandMode            = FMC_SDRAM_CMD_CLK_ENABLE;
	Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
	Command.AutoRefreshNumber      = 1;
	Command.ModeRegisterDefinition = 0;

	/* Send the command */
	HAL_SDRAM_SendCommand(sdramHandle, &Command, SDRAM_TIMEOUT);

	HAL_Delay(1);

	/* Configure a PALL (precharge all) command */
	Command.CommandMode            = FMC_SDRAM_CMD_PALL;
	Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
	Command.AutoRefreshNumber      = 1;
	Command.ModeRegisterDefinition = 0;

	/* Send the command */
	HAL_SDRAM_SendCommand(sdramHandle, &Command, SDRAM_TIMEOUT);

	/* Configure an Auto Refresh command */
	Command.CommandMode            = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
	Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
	Command.AutoRefreshNumber      = 8;
	Command.ModeRegisterDefinition = 0;

	/* Send the command */
	HAL_SDRAM_SendCommand(sdramHandle, &Command, SDRAM_TIMEOUT);

	/* Program the external memory mode register */
	tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_1          |\
					 SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |\
					 SDRAM_MODEREG_CAS_LATENCY_2           |\
					 SDRAM_MODEREG_OPERATING_MODE_STANDARD |\
					 SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

	Command.CommandMode            = FMC_SDRAM_CMD_LOAD_MODE;
	Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
	Command.AutoRefreshNumber      = 1;
	Command.ModeRegisterDefinition = tmpmrd;

	/* Send the command */
	HAL_SDRAM_SendCommand(sdramHandle, &Command, SDRAM_TIMEOUT);

	/* Set the refresh rate counter */
	/* Set the device refresh rate */
	HAL_SDRAM_ProgramRefreshRate(sdramHandle, RefreshCount);
}

void SDRAM_Initialization_sequence2(SDRAM_HandleTypeDef  *sdramHandle, uint32_t RefreshCount)
{

	__IO uint32_t tmpmrd = 0;
	FMC_SDRAM_CommandTypeDef Command;
	/* Configure a clock configuration enable command */
	Command.CommandMode            = FMC_SDRAM_CMD_CLK_ENABLE;
	Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK2;
	Command.AutoRefreshNumber      = 1;
	Command.ModeRegisterDefinition = 0;

	/* Send the command */
	HAL_SDRAM_SendCommand(sdramHandle, &Command, SDRAM_TIMEOUT);

	HAL_Delay(1);

	/* Configure a PALL (precharge all) command */
	Command.CommandMode            = FMC_SDRAM_CMD_PALL;
	Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK2;
	Command.AutoRefreshNumber      = 1;
	Command.ModeRegisterDefinition = 0;

	/* Send the command */
	HAL_SDRAM_SendCommand(sdramHandle, &Command, SDRAM_TIMEOUT);

	/* Configure an Auto Refresh command */
	Command.CommandMode            = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
	Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK2;
	Command.AutoRefreshNumber      = 8;
	Command.ModeRegisterDefinition = 0;

	/* Send the command */
	HAL_SDRAM_SendCommand(sdramHandle, &Command, SDRAM_TIMEOUT);

	/* Program the external memory mode register */
	tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_1          |\
					 SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |\
					 SDRAM_MODEREG_CAS_LATENCY_2           |\
					 SDRAM_MODEREG_OPERATING_MODE_STANDARD |\
					 SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

	Command.CommandMode            = FMC_SDRAM_CMD_LOAD_MODE;
	Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK2;
	Command.AutoRefreshNumber      = 1;
	Command.ModeRegisterDefinition = tmpmrd;

	/* Send the command */
	HAL_SDRAM_SendCommand(sdramHandle, &Command, SDRAM_TIMEOUT);

	/* Set the refresh rate counter */
	/* Set the device refresh rate */
	HAL_SDRAM_ProgramRefreshRate(sdramHandle, RefreshCount);
}

/*****************	SRAM *******************/
void SRAM1_Initialization(SRAM_HandleTypeDef* hsram)
{
	/* SRAM device configuration */
	hsram->Instance  = FMC_NORSRAM_DEVICE;
	hsram->Extended  = FMC_NORSRAM_EXTENDED_DEVICE;

	FMC_NORSRAM_TimingTypeDef SRAM_Timing;

	/* SRAM device configuration */
	SRAM_Timing.AddressSetupTime       = 4;
	SRAM_Timing.AddressHoldTime        = 1;
	SRAM_Timing.DataSetupTime          = 2;
	SRAM_Timing.BusTurnAroundDuration  = 1;
	SRAM_Timing.CLKDivision            = 2;
	SRAM_Timing.DataLatency            = 2;
	SRAM_Timing.AccessMode             = FMC_ACCESS_MODE_A;

	hsram->Init.NSBank             = FMC_NORSRAM_BANK1;
	hsram->Init.DataAddressMux     = FMC_DATA_ADDRESS_MUX_DISABLE;
	hsram->Init.MemoryType         = FMC_MEMORY_TYPE_SRAM;
	hsram->Init.MemoryDataWidth    = SRAM_MEMORY_WIDTH;
	hsram->Init.BurstAccessMode    = FMC_BURST_ACCESS_MODE_DISABLE;
	hsram->Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
	hsram->Init.WaitSignalActive   = FMC_WAIT_TIMING_BEFORE_WS;
	hsram->Init.WriteOperation     = FMC_WRITE_OPERATION_ENABLE;
	hsram->Init.WaitSignal         = FMC_WAIT_SIGNAL_DISABLE;
	hsram->Init.ExtendedMode       = FMC_EXTENDED_MODE_DISABLE;
	hsram->Init.AsynchronousWait   = FMC_ASYNCHRONOUS_WAIT_DISABLE;
	hsram->Init.WriteBurst         = FMC_WRITE_BURST_DISABLE;
	hsram->Init.ContinuousClock    = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
	hsram->Init.WriteFifo          = FMC_WRITE_FIFO_DISABLE;
	hsram->Init.PageSize           = FMC_PAGE_SIZE_NONE;

	/* Initialize the SRAM controller */
	if(HAL_SRAM_Init(hsram, &SRAM_Timing, &SRAM_Timing) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}

}

void SRAM2_Initialization(SRAM_HandleTypeDef* hsram)
{
	/* SRAM device configuration */
	hsram->Instance  = FMC_NORSRAM_DEVICE;
	hsram->Extended  = FMC_NORSRAM_EXTENDED_DEVICE;

	FMC_NORSRAM_TimingTypeDef SRAM_Timing;

	/* SRAM device configuration */
	SRAM_Timing.AddressSetupTime       = 4;
	SRAM_Timing.AddressHoldTime        = 1;
	SRAM_Timing.DataSetupTime          = 2;
	SRAM_Timing.BusTurnAroundDuration  = 1;
	SRAM_Timing.CLKDivision            = 2;
	SRAM_Timing.DataLatency            = 2;
	SRAM_Timing.AccessMode             = FMC_ACCESS_MODE_A;

	hsram->Init.NSBank             = FMC_NORSRAM_BANK3;
	hsram->Init.DataAddressMux     = FMC_DATA_ADDRESS_MUX_DISABLE;
	hsram->Init.MemoryType         = FMC_MEMORY_TYPE_SRAM;
	hsram->Init.MemoryDataWidth    = SRAM_MEMORY_WIDTH;
	hsram->Init.BurstAccessMode    = FMC_BURST_ACCESS_MODE_DISABLE;
	hsram->Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
	hsram->Init.WaitSignalActive   = FMC_WAIT_TIMING_BEFORE_WS;
	hsram->Init.WriteOperation     = FMC_WRITE_OPERATION_ENABLE;
	hsram->Init.WaitSignal         = FMC_WAIT_SIGNAL_DISABLE;
	hsram->Init.ExtendedMode       = FMC_EXTENDED_MODE_DISABLE;
	hsram->Init.AsynchronousWait   = FMC_ASYNCHRONOUS_WAIT_DISABLE;
	hsram->Init.WriteBurst         = FMC_WRITE_BURST_DISABLE;
	hsram->Init.ContinuousClock    = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
	hsram->Init.WriteFifo          = FMC_WRITE_FIFO_DISABLE;
	hsram->Init.PageSize           = FMC_PAGE_SIZE_NONE;

	/* Initialize the SRAM controller */
	if(HAL_SRAM_Init(hsram, &SRAM_Timing, &SRAM_Timing) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}

}

void SRAM1_Test_Sequence()
{
	/* Clear the buffer to write */
	SRAM_Fill_Buffer(sram_aTxBuffer, SRAM_BUFFER_SIZE, 0xC20F);
	/* Clear the buffer to read */
	SRAM_Fill_Buffer(sram_aRxBuffer, SRAM_BUFFER_SIZE, 0x0000);

	/* Write data to the SRAM memory */
	HAL_SRAM_Write_16b(&hsram1, (uint32_t *)(SRAM_BANK_ADDR1 + SRAM_WRITE_READ_ADDR), (uint16_t *)sram_aTxBuffer, SRAM_BUFFER_SIZE*2);//Size x2 to write 32bit data size with 16bit Function
	/* Read back data from the SRAM memory */
	HAL_SRAM_Read_16b(&hsram1, (uint32_t *)(SRAM_BANK_ADDR1 + SRAM_WRITE_READ_ADDR), (uint16_t *)sram_aRxBuffer, SRAM_BUFFER_SIZE*2);

	/* Checking data integrity */
	sram_uwWriteReadStatus = SRAM_Buffercmp(sram_aTxBuffer, sram_aRxBuffer, SRAM_BUFFER_SIZE);

	if(sram_uwWriteReadStatus) /* NG */
	{
		printf("SRAM1 Test NG\n\r");
	}
	else /* OK */
	{
		printf("SRAM1 Test PASS\n\r");
	}

}
void SRAM2_Test_Sequence()
{
	/* Clear the buffer to write */
	SRAM_Fill_Buffer(sram_aTxBuffer, SRAM_BUFFER_SIZE, 0xC20F);
	/* Clear the buffer to read */
	SRAM_Fill_Buffer(sram_aRxBuffer, SRAM_BUFFER_SIZE, 0x0000);

	/* Write data to the SRAM memory */
	HAL_SRAM_Write_16b(&hsram1, (uint32_t *)(SRAM_BANK_ADDR2 + SRAM_WRITE_READ_ADDR), (uint16_t *)sram_aTxBuffer, SRAM_BUFFER_SIZE*2);//Size x2 to write 32bit data size with 16bit Function
	/* Read back data from the SRAM memory */
	HAL_SRAM_Read_16b(&hsram1, (uint32_t *)(SRAM_BANK_ADDR2 + SRAM_WRITE_READ_ADDR), (uint16_t *)sram_aRxBuffer, SRAM_BUFFER_SIZE*2);

	/* Checking data integrity */
	sram_uwWriteReadStatus = SRAM_Buffercmp(sram_aTxBuffer, sram_aRxBuffer, SRAM_BUFFER_SIZE);

	if(sram_uwWriteReadStatus) /* NG */
	{
	  printf("SRAM2 Test NG\n\r");
	}
	else /* OK */
	{
	  printf("SRAM2 Test PASS\n\r");
	}

}

/*****************	NOR FLASH *******************/
void NOR_Flash_Initialization(NOR_HandleTypeDef* hnor)
{
	/* NOR device configuration */
	FMC_NORSRAM_TimingTypeDef NOR_Timing;
	hnor->Instance  = FMC_NORSRAM_DEVICE;
	hnor->Extended  = FMC_NORSRAM_EXTENDED_DEVICE;

	NOR_Timing.AddressSetupTime      = 15;
	NOR_Timing.AddressHoldTime       = 15;
	NOR_Timing.DataSetupTime         = 255;
	NOR_Timing.BusTurnAroundDuration = 15;
	NOR_Timing.CLKDivision           = 16;
	NOR_Timing.DataLatency           = 17;
	NOR_Timing.AccessMode            = FMC_ACCESS_MODE_B;

	hnor->Init.NSBank             = FMC_NORSRAM_BANK4;
	hnor->Init.DataAddressMux     = FMC_DATA_ADDRESS_MUX_DISABLE;
	hnor->Init.MemoryType         = FMC_MEMORY_TYPE_NOR;
	hnor->Init.MemoryDataWidth    = NOR_MEMORY_WIDTH;
	hnor->Init.BurstAccessMode    = FMC_BURST_ACCESS_MODE_DISABLE;
	hnor->Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
	hnor->Init.WaitSignalActive   = FMC_WAIT_TIMING_BEFORE_WS;
	hnor->Init.WriteOperation     = FMC_WRITE_OPERATION_ENABLE;
	hnor->Init.WaitSignal         = FMC_WAIT_SIGNAL_ENABLE;
	hnor->Init.ExtendedMode       = FMC_EXTENDED_MODE_DISABLE;
	hnor->Init.AsynchronousWait   = FMC_ASYNCHRONOUS_WAIT_ENABLE;
	hnor->Init.WriteBurst         = FMC_WRITE_BURST_DISABLE;
	hnor->Init.ContinuousClock    = FMC_CONTINUOUS_CLOCK_SYNC_ASYNC;

	/* Initialize the NOR controller */
	if(HAL_NOR_Init(hnor, &NOR_Timing, &NOR_Timing) != HAL_OK)
	{
		/* Initialization Error */
		printf("HAL_NOR_Init ERR \n\r");
		Error_Handler();
	}

	NOR_IDTypeDef norID;
	if(HAL_NOR_Read_ID(hnor, &norID) != HAL_OK)
	{
		printf("HAL_NOR_Read_ID ERR \n\r");
	}
	else
	{
		printf("HAL_NOR_Read_ID  ID0:0x%X, ID1:0x%X, ID2:0x%X, ID3:0x%X \n\r",
				norID.Manufacturer_Code,norID.Device_Code1,norID.Device_Code2,norID.Device_Code3);
		//S29GL512 : 0x90, 0x01, 0x2227, 0x2223

	}

	HAL_NOR_WriteOperation_Enable(hnor);
}

void NOR_Flash_Test_Sequence()
{
	/* Fill the buffer to write */
	NOR_Fill_Buffer(nor_aTxBuffer, NOR_BUFFER_SIZE, 0xD20F);
	printf("NOR_Fill_Buffer OK\n\r");

	HAL_NOR_ReturnToReadMode(&hnor3);

	/* Erase block */
	if(HAL_NOR_Erase_Block(&hnor3, NOR_WRITE_READ_ADDR, NOR_BANK_ADDR) !=HAL_OK)
	{
		printf("HAL_NOR_Erase_Block ERR\n\r");
	}
	else
	{
		printf("HAL_NOR_Erase_Block OK\n\r");
	}

	/* Wait until NOR is ready */
	if(HAL_NOR_GetStatus(&hnor3, NOR_BANK_ADDR, NOR_ERASE_TIMEOUT) != HAL_NOR_STATUS_SUCCESS)
	{
		printf("HAL_NOR_STATUS_ERROR:HAL_NOR_Erase_Block\n\r");
		return;
	}
	else
	{
		printf("HAL_NOR_STATUS_SUCCESS:HAL_NOR_Erase_Block\n\r");
	}

	/* Write data to the NOR memory */
	for (nor_uwIndex = 0; nor_uwIndex < NOR_BUFFER_SIZE; nor_uwIndex++)
	{
		/* Write data to NOR */
		HAL_NOR_Program(&hnor3, (uint32_t *)(NOR_BANK_ADDR + NOR_WRITE_READ_ADDR + 2*nor_uwIndex), &nor_aTxBuffer[nor_uwIndex]);

		/* Read NOR device status */
		if(HAL_NOR_GetStatus(&hnor3, NOR_BANK_ADDR, NOR_PROGRAM_TIMEOUT) != HAL_NOR_STATUS_SUCCESS)
		{
			printf("HAL_NOR_STATUS_ERROR:HAL_NOR_Program\n\r");
			return;
		}
	}

	/* Read back data from the NOR memory */
	if(HAL_NOR_ReadBuffer(&hnor3, NOR_BANK_ADDR + NOR_WRITE_READ_ADDR, &nor_aRxBuffer[0], NOR_BUFFER_SIZE) != HAL_OK)
	{
		printf("HAL_NOR_STATUS_ERROR:HAL_NOR_ReadBuffer\n\r");
		return;
	}

	/* Checking data integrity */
	nor_uwWriteReadStatus = NOR_Buffercmp(nor_aTxBuffer, nor_aRxBuffer, NOR_BUFFER_SIZE);

	if (nor_uwWriteReadStatus != PASSED)
	{
	  printf("NOR Flash Test NG\n\r");
	}
	else /* OK */
	{
	  printf("NOR Flash Test PASS\n\r");
	}
	while(1);
}

static void NAND_Fill_Buffer(uint8_t *pBuffer, uint32_t uwBufferLenght, uint32_t uwOffset)
{
	uint32_t index = 0;

	/* Put in global buffer same values */
	for (index = 0; index < uwBufferLenght; index++ )
	{
		pBuffer[index] = index + uwOffset;
	}
}

static void SRAM_Fill_Buffer(uint32_t *pBuffer, uint32_t uwBufferLength, uint16_t uwOffset)
{
	uint16_t tmpIndex = 0;

	/* Put in global buffer different values */
	for (tmpIndex = 0; tmpIndex < uwBufferLength; tmpIndex++)
	{
		pBuffer[tmpIndex] = tmpIndex + uwOffset;
	}
}

static uint8_t SRAM_Buffercmp(uint32_t* pBuffer1, uint32_t* pBuffer2, uint16_t BufferLength)
{
	while (BufferLength--)
	{
		if (*pBuffer1 != *pBuffer2)
		{
		  return 1;
		}

		pBuffer1++;
		pBuffer2++;
	}

	return 0;
}

static void NOR_Fill_Buffer(uint16_t *pBuffer, uint32_t uwBufferLenght, uint16_t uwOffset)
{
	uint16_t tmpIndex = 0;

	/* Put in global buffer different values */
	for (tmpIndex = 0; tmpIndex < uwBufferLenght; tmpIndex++ )
	{
		pBuffer[tmpIndex] = tmpIndex + uwOffset;
	}
}

static TestStatus NOR_Buffercmp(uint16_t* pBuffer1, uint16_t* pBuffer2, uint16_t BufferLength)
{
	while (BufferLength--)
	{
		if (*pBuffer1 != *pBuffer2)
		{
		  return FAILED;
		}

		pBuffer1++;
		pBuffer2++;
	}

	return PASSED;
}

static TestStatus NAND_Buffercmp(uint8_t* pBuffer, uint8_t* pBuffer1, uint32_t uwBufferLenght)
{
	uint32_t counter = 0;
	while(uwBufferLenght--)
	{
		if(*pBuffer != *pBuffer1)
		{
			return FAILED;
		}

		pBuffer++;
		pBuffer1++;
		counter++;
	}

	return PASSED;
}

static void NAND_GetAddress (uint32_t Address, NAND_AddressTypeDef *pNandAddress)
{
	pNandAddress->Page  = (Address % (NAND_BLOCK_SIZE * (NAND_PAGE_SIZE + NAND_SPARE_AREA_SIZE))) / (NAND_PAGE_SIZE + NAND_SPARE_AREA_SIZE);
	pNandAddress->Block = (Address % (NAND_PLANE_SIZE * NAND_BLOCK_SIZE * (NAND_PAGE_SIZE + NAND_SPARE_AREA_SIZE))) / (NAND_BLOCK_SIZE * (NAND_PAGE_SIZE + NAND_SPARE_AREA_SIZE));
	pNandAddress->Plane = Address / (NAND_PLANE_SIZE * NAND_BLOCK_SIZE * (NAND_PAGE_SIZE + NAND_SPARE_AREA_SIZE));
}



void NAND_Initialization_sequence(NAND_HandleTypeDef *hnand)
{
	FMC_NAND_PCC_TimingTypeDef ComSpaceTiming = {0};
	FMC_NAND_PCC_TimingTypeDef AttSpaceTiming = {0};

	hnand1.Instance = FMC_NAND_DEVICE;
	/* hnand1.Init */
	hnand1.Init.NandBank = FMC_NAND_BANK3;
	hnand1.Init.Waitfeature = FMC_NAND_WAIT_FEATURE_ENABLE;
	hnand1.Init.MemoryDataWidth = FMC_NAND_MEM_BUS_WIDTH_8;
	hnand1.Init.EccComputation = FMC_NAND_ECC_ENABLE;
	hnand1.Init.ECCPageSize = FMC_NAND_ECC_PAGE_SIZE_2048BYTE;
	hnand1.Init.TCLRSetupTime = 0;
	hnand1.Init.TARSetupTime = 0;
	/* hnand1.Config */
	hnand1.Config.PageSize = NAND_PAGE_SIZE;
	hnand1.Config.BlockSize = NAND_BLOCK_SIZE;
	hnand1.Config.BlockNbr = NAND_BLOCK_NBR;
	hnand1.Config.PlaneSize = NAND_PLANE_SIZE;
	hnand1.Config.PlaneNbr = NAND_PLANE_NBR;
	hnand1.Config.SpareAreaSize = NAND_SPARE_AREA_SIZE;

	hnand1.Config.ExtraCommandEnable = DISABLE;
	/* ComSpaceTiming */
	ComSpaceTiming.SetupTime = 2;
	ComSpaceTiming.WaitSetupTime = 3;
	ComSpaceTiming.HoldSetupTime = 6;
	ComSpaceTiming.HiZSetupTime = 2;
	/* AttSpaceTiming */
	AttSpaceTiming.SetupTime = 2;
	AttSpaceTiming.WaitSetupTime = 3;
	AttSpaceTiming.HoldSetupTime = 6;
	AttSpaceTiming.HiZSetupTime = 2;

	//FMC CLK around 50MHz is best, the most stable.

	if (HAL_NAND_Init(&hnand1, &ComSpaceTiming, &AttSpaceTiming) != HAL_OK)
	{
	  printf("NAND_Init_ERR\n\r");
	  Error_Handler( );

	}else{
	  printf("NAND_Init OK\n\r");
	}

	HAL_NAND_Reset(&hnand1);
	printf("NAND_RESET OK\n\r");
	HAL_Delay(10);


	/* Read NAND memory ID */
	if(HAL_NAND_Read_ID(&hnand1, &NAND_Id) != HAL_OK)
	{
		/* NAND read ID Error */
		printf("Cannot NAND Read ID ERR!!\n\r");
		//Error_Handler();
	}else
	{
		printf("Maker_Id: 0x%X, Device_Id: 0x%X, Third_Id: 0x%X, Fourth_Id: 0x%X \n\r",
					NAND_Id.Maker_Id,NAND_Id.Device_Id,NAND_Id.Third_Id,NAND_Id.Fourth_Id);
	}

	HAL_Delay(10);


	/* Test the NAND ID correctness */
	if((NAND_Id.Maker_Id != NAND_MAKERID) || (NAND_Id.Device_Id != NAND_DEVICEID))
	{
		/* NAND ID not correct */
		printf("NAND ID is not correct!!\n\r");
	}

}

void NAND_Test_sequence(NAND_HandleTypeDef *hnand)
{
	HAL_Delay(10);
	/* Convert Address to NAND address */
	NAND_GetAddress(WRITE_READ_ADDR, &NAND_Address);

	printf("Get NAND Addr Block:0x%X, Page:0x%X, Plane:0x%X \n\r",NAND_Address.Block,NAND_Address.Page,NAND_Address.Plane);

	/* Erase NAND memory */
	if(HAL_NAND_Erase_Block(&hnand1, &NAND_Address) != HAL_OK)
	{
		printf("NAND erase error!!\n\r");
	}
	else
	{
		printf("NAND erase OK!!\n\r");
	}

	HAL_Delay(10);


	while(HAL_NAND_Read_Status(&hnand1)!=NAND_READY);

	/* NAND memory read/write access */
	/* Fill the buffer to write */
	NAND_Fill_Buffer(nand_aTxBuffer, NAND_BUFFER_SIZE, 0xD210);

	HAL_Delay(10);

	while(HAL_NAND_Read_Status(&hnand1)!=NAND_READY);

	/* Write data to the NAND memory */
	#ifdef ECC_ENABLE
	HAL_NAND_ECC_Enable(&hnand1);
	#endif


	if(HAL_NAND_Write_Page_8b(&hnand1, &NAND_Address, nand_aTxBuffer, NAND_NB_PAGE) != HAL_OK)
	{
		printf("NAND HAL_NAND_Write_Page_8b error!!\n\r");
	}
	uint32_t ecc[3]={0};


	#ifdef ECC_ENABLE
	HAL_NAND_GetECC(&hnand1, &ecc[0], HAL_MAX_DELAY);
	HAL_NAND_ECC_Disable(&hnand1);
	HAL_NAND_Write_SpareArea_8b(&hnand1, &NAND_Address, (uint8_t *)&ecc[0], 1);
	#endif


	/* Read back data from the NAND memory */
	#ifdef ECC_ENABLE
	HAL_NAND_ECC_Enable(&hnand1);
	#endif
	if(HAL_NAND_Read_Page_8b(&hnand1, &NAND_Address, nand_aRxBuffer, NAND_NB_PAGE) != HAL_OK)
	{
		printf("NAND HAL_NAND_Read_Page_8b error!!\n\r");
	}

	#ifdef ECC_ENABLE
	HAL_NAND_GetECC(&hnand1, &ecc[1], HAL_MAX_DELAY);
	HAL_NAND_ECC_Disable(&hnand1);
	HAL_NAND_Read_SpareArea_8b(&hnand1, &NAND_Address, (uint8_t *)&ecc[2], 1);
	#endif


	if (ecc[1] != ecc[2])printf("ECC def= 0x%lx, rECC= 0x%lx, gECC= 0x%lx\n",ecc[0],ecc[1],ecc[2]);

	if(ecc[0]==ecc[2]&& ecc[1]==0 )printf("NO ERR\n\r");

	HAL_Delay(10);
	if(NAND_Buffercmp(nand_aTxBuffer, nand_aRxBuffer, NAND_BUFFER_SIZE) != PASSED)
	{
		printf("NAND Buffercmp error!!\n\r");
		for(int i=0; i<NAND_BUFFER_SIZE;i++)
		{
			if(nand_aTxBuffer[i]!=nand_aRxBuffer[i])
			{
				printf("DIFF  i: 0x%X, TX: 0x%X, RX: 0x%X \n\r",i,nand_aTxBuffer[i],nand_aRxBuffer[i]);
				while(1);
			}
		}
	}
	else
	{
		printf("NAND Buffercmp OK!!\n\r");
	}

}



static void MPU_Config(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//MPU_Config();//Must Enable in case of using D Cache
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
  MX_ETH_Init();
  MX_FMC_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("Memory Shield Init OK\n\r");

  NAND_Initialization_sequence(&hnand1);
  NAND_Test_sequence(&hnand1);

  SRAM1_Initialization(&hsram1);
  SRAM2_Initialization(&hsram2);
  SRAM1_Test_Sequence();
  SRAM2_Test_Sequence();

  NOR_Flash_Initialization(&hnor3);
  NOR_Flash_Test_Sequence();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable 
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_FMC;
  PeriphClkInitStruct.FmcClockSelection = RCC_FMCCLKSOURCE_D1HCLK;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  heth.Init.MACAddr[0] =   0x00;
  heth.Init.MACAddr[1] =   0x80;
  heth.Init.MACAddr[2] =   0xE1;
  heth.Init.MACAddr[3] =   0x00;
  heth.Init.MACAddr[4] =   0x00;
  heth.Init.MACAddr[5] =   0x00;
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */
    
  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */
	HAL_NAND_MspInit(&hnand1);
  /* USER CODE END FMC_Init 0 */

  FMC_NORSRAM_TimingTypeDef Timing = {0};
  FMC_NORSRAM_TimingTypeDef ExtTiming = {0};
  FMC_NAND_PCC_TimingTypeDef ComSpaceTiming = {0};
  FMC_NAND_PCC_TimingTypeDef AttSpaceTiming = {0};
  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FMC_NORSRAM_DEVICE;
  hsram1.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hsram1.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram1.Init.WriteFifo = FMC_WRITE_FIFO_ENABLE;
  hsram1.Init.PageSize = FMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 2;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 1;
  Timing.BusTurnAroundDuration = 3;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Perform the SRAM2 memory initialization sequence
  */
  hsram2.Instance = FMC_NORSRAM_DEVICE;
  hsram2.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram2.Init */
  hsram2.Init.NSBank = FMC_NORSRAM_BANK3;
  hsram2.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  hsram2.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;
  hsram2.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram2.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hsram2.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram2.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hsram2.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
  hsram2.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hsram2.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
  hsram2.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram2.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hsram2.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram2.Init.WriteFifo = FMC_WRITE_FIFO_ENABLE;
  hsram2.Init.PageSize = FMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 2;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 1;
  Timing.BusTurnAroundDuration = 3;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram2, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Perform the NOR3 memory initialization sequence
  */
  hnor3.Instance = FMC_NORSRAM_DEVICE;
  hnor3.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hnor3.Init */
  hnor3.Init.NSBank = FMC_NORSRAM_BANK4;
  hnor3.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  hnor3.Init.MemoryType = FMC_MEMORY_TYPE_NOR;
  hnor3.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  hnor3.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hnor3.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hnor3.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hnor3.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
  hnor3.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hnor3.Init.ExtendedMode = FMC_EXTENDED_MODE_ENABLE;
  hnor3.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_ENABLE;
  hnor3.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hnor3.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hnor3.Init.WriteFifo = FMC_WRITE_FIFO_ENABLE;
  hnor3.Init.PageSize = FMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 7;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 2;
  Timing.BusTurnAroundDuration = 3;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FMC_ACCESS_MODE_B;
  /* ExtTiming */
  ExtTiming.AddressSetupTime = 0;
  ExtTiming.AddressHoldTime = 15;
  ExtTiming.DataSetupTime = 3;
  ExtTiming.BusTurnAroundDuration = 3;
  ExtTiming.CLKDivision = 16;
  ExtTiming.DataLatency = 17;
  ExtTiming.AccessMode = FMC_ACCESS_MODE_B;

  if (HAL_NOR_Init(&hnor3, &Timing, &ExtTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Perform the NAND1 memory initialization sequence
  */
  hnand1.Instance = FMC_NAND_DEVICE;
  /* hnand1.Init */
  hnand1.Init.NandBank = FMC_NAND_BANK3;
  hnand1.Init.Waitfeature = FMC_NAND_WAIT_FEATURE_ENABLE;
  hnand1.Init.MemoryDataWidth = FMC_NAND_MEM_BUS_WIDTH_8;
  hnand1.Init.EccComputation = FMC_NAND_ECC_ENABLE;
  hnand1.Init.ECCPageSize = FMC_NAND_ECC_PAGE_SIZE_1024BYTE;
  hnand1.Init.TCLRSetupTime = 0;
  hnand1.Init.TARSetupTime = 0;
  /* hnand1.Config */
  hnand1.Config.PageSize = 2048;
  hnand1.Config.SpareAreaSize = 32;
  hnand1.Config.BlockSize = 64;
  hnand1.Config.BlockNbr = 2048;
  hnand1.Config.PlaneNbr = 4;
  hnand1.Config.PlaneSize = 512;
  hnand1.Config.ExtraCommandEnable = DISABLE;
  /* ComSpaceTiming */
  ComSpaceTiming.SetupTime = 0;
  ComSpaceTiming.WaitSetupTime = 1;
  ComSpaceTiming.HoldSetupTime = 2;
  ComSpaceTiming.HiZSetupTime = 1;
  /* AttSpaceTiming */
  AttSpaceTiming.SetupTime = 0;
  AttSpaceTiming.WaitSetupTime = 1;
  AttSpaceTiming.HoldSetupTime = 2;
  AttSpaceTiming.HiZSetupTime = 1;

  if (HAL_NAND_Init(&hnand1, &ComSpaceTiming, &AttSpaceTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_9;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_13;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_2;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 6;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 6;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Perform the SDRAM2 memory initialization sequence
  */
  hsdram2.Instance = FMC_SDRAM_DEVICE;
  /* hsdram2.Init */
  hsdram2.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram2.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_9;
  hsdram2.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_13;
  hsdram2.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram2.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram2.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_2;
  hsdram2.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram2.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram2.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram2.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 6;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 6;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram2, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */
  SDRAM_Initialization_sequence1(&hsdram1, SDRAM_REFRESH_COUNT);
  SDRAM_Initialization_sequence2(&hsdram2, SDRAM_REFRESH_COUNT);
  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU attributes as Device not cacheable
     for ETH DMA descriptors */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Configure the MPU attributes as Normal Non Cacheable
     for LwIP RAM heap which contains the Tx buffers */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x30044000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Configure the MPU attributes  for NAND Flash */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x80000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Configure the MPU attributes  for NOR Flash */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x60000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER3;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
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

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
