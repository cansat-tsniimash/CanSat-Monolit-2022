#include <stdio.h>

#include <stm32f4xx_hal.h>

#include "lsm6ds3_reg.h"
#include "bme280.h"


static void bme_spi_cs_down(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
}


static void bme_spi_cs_up(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
}



static BME280_INTF_RET_TYPE bme_spi_read(
		uint8_t reg_addr, uint8_t * data, uint32_t data_len, void *intf_ptr
)
{
	extern SPI_HandleTypeDef hspi1;

	bme_spi_cs_down();
	reg_addr |= (1 << 7);
	HAL_SPI_Transmit(&hspi1, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, data, data_len, HAL_MAX_DELAY);
	bme_spi_cs_up();

	return 0;
}


static BME280_INTF_RET_TYPE bme_spi_write(
		uint8_t reg_addr, const uint8_t * data, uint32_t data_len, void *intf_ptr
)
{
	extern SPI_HandleTypeDef hspi1;

	bme_spi_cs_down();
	reg_addr &= ~(1 << 7);
	HAL_SPI_Transmit(&hspi1, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)data, data_len, HAL_MAX_DELAY);
	bme_spi_cs_up();

	return 0;
}


static void bme_delay_us(uint32_t period, void *intf_ptr)
{
	if (period < 1000)
		period = 1;
	else
		period = period / 1000;

	HAL_Delay(period);
}





static int32_t lsm_i2c_read(void *handle, uint8_t reg_addr, uint8_t * data, uint16_t data_len)
{
	extern I2C_HandleTypeDef hi2c1;
	HAL_I2C_Mem_Read(&hi2c1, LSM6DS3_I2C_ADD_H, reg_addr, 1, data, data_len, HAL_MAX_DELAY);
	return 0;
}


static int32_t lsm_i2c_write(void * handle, uint8_t reg_addr, const uint8_t * data, uint16_t data_len)
{
	extern I2C_HandleTypeDef hi2c1;
	HAL_I2C_Mem_Write(&hi2c1, LSM6DS3_I2C_ADD_H, reg_addr, 1, (uint8_t*)data, data_len, HAL_MAX_DELAY);
	return 0;
}



int app_main(void)
{
	// Настройка bme280 =-=-=-=-=-=-=-=-=-=-=-=-
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	struct bme280_dev bme = {0};
	bme.intf = BME280_SPI_INTF;
	bme.intf_ptr = NULL;
	bme.read = bme_spi_read;
	bme.write = bme_spi_write;
	bme.delay_us = bme_delay_us;

	int rc = bme280_soft_reset(&bme);
	printf("bme280 reset rc = %d\n", (int)rc);

	rc = bme280_init(&bme);
	printf("bme280 init rc = %d\n", (int)rc);

	bme.settings.osr_h = BME280_OVERSAMPLING_1X;
	bme.settings.osr_p = BME280_OVERSAMPLING_16X;
	bme.settings.osr_t = BME280_OVERSAMPLING_2X;
	bme.settings.filter = BME280_FILTER_COEFF_16;
	bme.settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

	uint8_t settings_sel;
	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_FILTER_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	rc = bme280_set_sensor_settings(settings_sel, &bme);
	printf("bme280 settings set rc = %d\n", rc);
	rc = bme280_set_sensor_mode(BME280_NORMAL_MODE, &bme);
	printf("bme280 set sensor mode rc = %d\n", rc);


	// Настройка lsm6ds3 =-=-=-=-=-=-=-=-=-=-=-=-
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	stmdev_ctx_t ctx = {0};
	ctx.handle = NULL;
	ctx.read_reg = lsm_i2c_read;
	ctx.write_reg = lsm_i2c_write;

	uint8_t whoami = 0x00;
	lsm6ds3_device_id_get(&ctx, &whoami);
	printf("got lsm6ds3 whoami 0x%02X, expected 0x%02X\n", (int)whoami, (int)LSM6DS3_ID);

	lsm6ds3_reset_set(&ctx, PROPERTY_ENABLE);
	HAL_Delay(100);

	lsm6ds3_xl_full_scale_set(&ctx, LSM6DS3_16g);
	lsm6ds3_xl_data_rate_set(&ctx, LSM6DS3_XL_ODR_104Hz);

	lsm6ds3_gy_full_scale_set(&ctx, LSM6DS3_2000dps);
	lsm6ds3_gy_data_rate_set(&ctx, LSM6DS3_GY_ODR_104Hz);

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

		// Вывод
		printf(
			"t = %8.4f; acc = %10.4f,%10.4f,%10.4f; gyro=%10.4f,%10.4f,%10.4f" " ||| ", //\n",
			temperature_celsius,
			acc_g[0], acc_g[1], acc_g[2],
			gyro_dps[0], gyro_dps[1], gyro_dps[2]
		);


		// Чтение данные из bme280
		// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

		//bme280_set_sensor_mode(BME280_FORCED_MODE, &bme);
		HAL_Delay(10);

		struct bme280_data comp_data;
		rc = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme);

		// Печать
		printf(
			"temp = %8.4f; pressure = %10.4f; hum = %10.4f\n",
			(float)comp_data.temperature,
			(float)comp_data.pressure,
			(float)comp_data.humidity
		);

		//HAL_Delay(100);
	}

	return 0;
}
