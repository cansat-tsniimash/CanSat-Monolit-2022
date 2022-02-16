/*
 * app_main.c
 *SSSS
 *  Created on: Jan 22, 2022
 *      Author: User
 */
#include <stdio.h>
#include "lsm6ds3_reg.h"
#include "stm32f4xx_hal.h"
#include "lis3mdl_reg.h"

extern SPI_HandleTypeDef hspi1;

static int32_t lsmd6s3_write(void * d, uint8_t reg_addr, const uint8_t * data, uint16_t data_size)
{
	HAL_SPI_DeInit(&hspi1);
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	HAL_SPI_Init(&hspi1);


	reg_addr=reg_addr&~(1<<7);

	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)data, data_size, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
	return 0;
}


static int32_t lsm6ds3_read(void * d, uint8_t reg_addr, uint8_t * data, uint16_t data_size)
{
	HAL_SPI_DeInit(&hspi1);
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	HAL_SPI_Init(&hspi1);

	reg_addr=reg_addr|(1<<7);

	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, data, data_size, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
	return 0;
}

static int32_t lsm303c_write(void * d, uint8_t reg_addr, const uint8_t * data, uint16_t data_size)
{
	HAL_SPI_DeInit(&hspi1);
	hspi1.Init.Direction = SPI_DIRECTION_1LINE;
	HAL_SPI_Init(&hspi1);

	reg_addr=reg_addr&~(1<<7);
	reg_addr=reg_addr|(1<<6);
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)data, data_size, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
	return 0;
}


static int32_t lsm303c_read(void * d, uint8_t reg_addr, uint8_t * data, uint16_t data_size)
{
	HAL_SPI_DeInit(&hspi1);
	hspi1.Init.Direction = SPI_DIRECTION_1LINE;
	HAL_SPI_Init(&hspi1);


	reg_addr=reg_addr|(1<<7);
	reg_addr=reg_addr|(1<<6);
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, data, data_size, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
	return 0;
}


//static void lsm6ds3_MX_SPI1_Init(void)
//{
//
//  /* USER CODE BEGIN SPI1_Init 0 */
//
//  /* USER CODE END SPI1_Init 0 */
//
//  /* USER CODE BEGIN SPI1_Init 1 */
//
//  /* USER CODE END SPI1_Init 1 */
//  /* SPI1 parameter configuration*/
//  hspi1.Instance = SPI1;
//  hspi1.Init.Mode = SPI_MODE_MASTER;
//  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
//  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
//  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
//  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
//  hspi1.Init.NSS = SPI_NSS_SOFT;
//  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
//  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
//  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
//  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//  hspi1.Init.CRCPolynomial = 10;
//  if (HAL_SPI_Init(&hspi1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN SPI1_Init 2 */
//
//  /* USER CODE END SPI1_Init 2 */
//
//}

//static void lsm303c_MX_SPI1_Init(void)
//{
//
//  /* USER CODE BEGIN SPI1_Init 0 */
//
//  /* USER CODE END SPI1_Init 0 */
//
//  /* USER CODE BEGIN SPI1_Init 1 */
//
//  /* USER CODE END SPI1_Init 1 */
//  /* SPI1 parameter configuration*/
//  hspi1.Instance = SPI1;
//  hspi1.Init.Mode = SPI_MODE_MASTER;
//  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
//  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
//  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
//  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
//  hspi1.Init.NSS = SPI_NSS_SOFT;
//  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
//  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
//  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
//  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//  hspi1.Init.CRCPolynomial = 10;
//  if (HAL_SPI_Init(&hspi1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN SPI1_Init 2 */
//
//  /* USER CODE END SPI1_Init 2 */
//
//}



int app_main()
{
	// Настройка lsm6ds3 =-=-=-=-=-=-=-=-=-=-=-=-
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	stmdev_ctx_t ctx = {0};
	ctx.handle = NULL;
	ctx.read_reg = lsm6ds3_read;
	ctx.write_reg = lsmd6s3_write;

	uint8_t whoami = 0x00;
	lsm6ds3_device_id_get(&ctx, &whoami);
	printf("got lsm6ds3 whoami 0x%02X, expected 0x%02X\n", (int)whoami, (int)LSM6DS3_ID);

	lsm6ds3_reset_set(&ctx, PROPERTY_ENABLE);
	HAL_Delay(100);

	lsm6ds3_xl_full_scale_set(&ctx, LSM6DS3_16g);
	lsm6ds3_xl_data_rate_set(&ctx, LSM6DS3_XL_ODR_104Hz);

	lsm6ds3_gy_full_scale_set(&ctx, LSM6DS3_2000dps);
	lsm6ds3_gy_data_rate_set(&ctx, LSM6DS3_GY_ODR_104Hz);


	// Настройка lismdl =-=-=-=-=-=-=-=-=-=-=-=-
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	stmdev_ctx_t mag_ctx;
	mag_ctx.handle = NULL;
	mag_ctx.read_reg = lsm303c_read;
	mag_ctx.write_reg = lsm303c_write;

	uint8_t whoami1 = 0x00;
	lis3mdl_device_id_get(&mag_ctx, &whoami1);
	printf("got lismdl whoami 0x%02X, expected 0x%02X\n", (int)whoami1, (int)LIS3MDL_ID);

	lis3mdl_reset_set(&mag_ctx, PROPERTY_ENABLE);
	HAL_Delay(100);

	/* Enable Block Data Update */
	lis3mdl_block_data_update_set(&mag_ctx, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	lis3mdl_data_rate_set(&mag_ctx, LIS3MDL_HP_1Hz25);
	/* Set full scale */
	lis3mdl_full_scale_set(&mag_ctx, LIS3MDL_16_GAUSS);
	/* Enable temperature sensor */
	lis3mdl_temperature_meas_set(&mag_ctx, PROPERTY_ENABLE);
	/* Set device in continuous mode */
	lis3mdl_operating_mode_set(&mag_ctx, LIS3MDL_CONTINUOUS_MODE);



	while(1)
	{
		// Чтение данных из lsm6ds3
		// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		int16_t temperature_raw;
		int16_t acc_raw[3];
		int16_t gyro_raw[3];
		lsm6ds3_temperature_raw_get(&ctx, &temperature_raw);
		lsm6ds3_acceleration_raw_get(&ctx, acc_raw);
		lsm6ds3_angular_rate_raw_get(&ctx, gyro_raw);

		// Пересчет из попугаев в человеческие величины
		float temperature_celsius;
		float acc_g[3];
		float gyro_dps[3];
		temperature_celsius = lsm6ds3_from_lsb_to_celsius(temperature_raw);
		for (int i = 0; i < 3; i++)
		{
			acc_g[i] = lsm6ds3_from_fs16g_to_mg(acc_raw[i]) / 1000;
			gyro_dps[i] = lsm6ds3_from_fs2000dps_to_mdps(gyro_raw[i]) / 1000;
		}


		int16_t mag_raw[3];
		lis3mdl_magnetic_raw_get(&ctx, mag_raw);
		float x;
		float y;
		float z;
		x = lis3mdl_from_fs16_to_gauss(mag_raw[0]);
		y = lis3mdl_from_fs16_to_gauss(mag_raw[1]);
		z = lis3mdl_from_fs16_to_gauss(mag_raw[2]);


		// Вывод
		printf(
			"t = %+8.4f; acc = %+10.4f,%+10.4f,%+10.4f; gyro=%+10.4f,%+10.4f,%+10.4f ", //\n",
			temperature_celsius,
			acc_g[0], acc_g[1], acc_g[2],
			gyro_dps[0], gyro_dps[1], gyro_dps[2]
		);

		printf(
				"mag = %+2.8f, %+2.8f, %+2.8f",
				x, y, z
		);

		printf("\n");




		//HAL_Delay(100);
	}

	return 0;
}


