/*
 * app_main.c
 *SSSS
 *  Created on: Jan 22, 2022
 *      Author: User
 */
#include <stdio.h>

#include "lsm6ds3_reg.h"
#include "lis3mdl_reg.h"

#include "stm32f4xx_hal.h"


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


static int32_t lis3mdl_write(void * d, uint8_t reg_addr, const uint8_t * data, uint16_t data_size)
{
	HAL_SPI_DeInit(&hspi1);
	hspi1.Init.Direction = SPI_DIRECTION_1LINE;
	HAL_SPI_Init(&hspi1);

	reg_addr=reg_addr&~(1<<7);
	reg_addr=reg_addr|(1<<6);

	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)data, data_size, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
	return 0;
}


static int32_t lis3mdl_read(void * d, uint8_t reg_addr, uint8_t * data, uint16_t data_size)
{
	HAL_SPI_DeInit(&hspi1);
	hspi1.Init.Direction = SPI_DIRECTION_1LINE;
	HAL_SPI_Init(&hspi1);

	reg_addr=reg_addr|(1<<7);
	reg_addr=reg_addr|(1<<6);

	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, data, data_size, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
	return 0;
}


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
	mag_ctx.read_reg = lis3mdl_read;
	mag_ctx.write_reg = lis3mdl_write;

	// Это придется делать прямо сразу еще до всего
	// так как иначе он с ним общаться не сможет (судя по доке по-крайней мере)
	lis3mdl_spi_mode_set(&mag_ctx, LIS3MDL_SPI_3_WIRE);

	uint8_t whoami_mag = 0x00;
	lis3mdl_device_id_get(&mag_ctx, &whoami_mag);
	printf("got lsm303agr whoami 0x%02X, expected 0x%02X\n", (int)whoami_mag, (int)LIS3MDL_ID);

	// Убедились что датчик тот который нам нужен
	// Сбросим его
	lis3mdl_reset_set(&mag_ctx, PROPERTY_ENABLE);
	HAL_Delay(100);

	// Настраиваем
	// Обновление данных только целыми порциями
	lis3mdl_block_data_update_set(&mag_ctx, PROPERTY_ENABLE);
	// Без экономии энергии
	lis3mdl_fast_low_power_set(&mag_ctx, PROPERTY_DISABLE);
	// Диапазон измерения (внимание LSM303 умеет только 16G)
	lis3mdl_full_scale_set(&mag_ctx, LIS3MDL_16_GAUSS);
	// Частота опроса
	// внимание для LSM303 запрещены значения
	// LIS3MDL_LP_1kHz, LIS3MDL_MP_560Hz, LIS3MDL_HP_300Hz, LIS3MDL_UHP_155Hz
	lis3mdl_data_rate_set(&mag_ctx, LIS3MDL_UHP_80Hz);
	// Включаем температурный сенсор
	lis3mdl_temperature_meas_set(&mag_ctx, PROPERTY_ENABLE);
	// режим работы
	lis3mdl_operating_mode_set(&mag_ctx, LIS3MDL_CONTINUOUS_MODE);


	while(1)
	{
		// Чтение данных из lsm6ds3
		// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		int16_t temperature_raw_gyro;
		int16_t acc_raw[3];
		int16_t gyro_raw[3];
		lsm6ds3_temperature_raw_get(&ctx, &temperature_raw_gyro);
		lsm6ds3_acceleration_raw_get(&ctx, acc_raw);
		lsm6ds3_angular_rate_raw_get(&ctx, gyro_raw);

		// Пересчет из попугаев в человеческие величины
		float temperature_celsius_gyro;
		float acc_g[3];
		float gyro_dps[3];
		temperature_celsius_gyro = lsm6ds3_from_lsb_to_celsius(temperature_raw_gyro);
		for (int i = 0; i < 3; i++)
		{
			acc_g[i] = lsm6ds3_from_fs16g_to_mg(acc_raw[i]) / 1000;
			gyro_dps[i] = lsm6ds3_from_fs2000dps_to_mdps(gyro_raw[i]) / 1000;
		}


		// Чтение данных из lis3mdl
		// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		int16_t temperataure_raw_mag;
		int16_t mag_raw[3];
		float mag[3];
		float temperature_celsius_mag;
		lis3mdl_magnetic_raw_get(&mag_ctx, mag_raw);
		lis3mdl_temperature_raw_get(&mag_ctx, &temperataure_raw_mag);
		temperature_celsius_mag = lis3mdl_from_lsb_to_celsius(temperataure_raw_mag);
		for (int i = 0; i < 3; i++)
			mag[i] = lis3mdl_from_fs16_to_gauss(mag_raw[i]);


		// Вывод
		printf(
			"t = %+3.4f; acc = %+10.4f,%+10.4f,%+10.4f; gyro=%+10.4f,%+10.4f,%+10.4f ", //\n",
			temperature_celsius_gyro,
			acc_g[0], acc_g[1], acc_g[2],
			gyro_dps[0], gyro_dps[1], gyro_dps[2]
		);

		printf(
				"t = %+3.4f; %mag = %+2.8f, %+2.8f, %+2.8f",
				temperature_celsius_mag, mag[0], mag[1], mag[2]
		);

		printf("\n");

		//HAL_Delay(100);
	}

	return 0;
}


