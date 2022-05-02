#include <stm32f4xx_hal.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>


#include "fatfs.h"
#include "LSM6DS3/DLSM.h"
#include "BME280/DriverForBME280.h"
#include "nRF24L01_PL/nrf24_defs.h"
#include "nRF24L01_PL/nrf24_upper_api.h"
#include "nRF24L01_PL/nrf24_lower_api_stm32.h"
#include "nRF24L01_PL/nrf24_lower_api.h"

#include "Shift_Register/shift_reg.h"



extern SPI_HandleTypeDef hspi2;
extern  UART_HandleTypeDef huart1;

//радио: настройка радио-части(+) и настройка протокольной части(+), настройка пайпа(+), перегон в режим отправки (тх) (+); потом радио пакты пихаем в ФИФО(проверка на свободное место в ФИФО),
unsigned short Crc16(unsigned char *buf, unsigned short len)
{
	unsigned short crc = 0xFFFF;//переменная 16 бит = 2 байта
	unsigned char i; //переменная 8 бит = 1 байт
	while (len--)// проверка условия продолжения
	{
		crc ^= *buf++ << 8;
		for (i = 0; i < 8; i++)//цикл перебора полинома
			crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
	}
	return crc;//конец функции расчёта Crc16
}

int app_main(void)
{

	// Переменные для работы с бме и лсм
	struct bme280_dev bme = {0};
	stmdev_ctx_t ctx = {0};

// НАСТРОЙКА ЛСМ И БМЕ (СТРАКТЫ)
	shift_reg_t shift_reg_ = {
			.bus = &hspi2,
			.latch_port= GPIOC,
			.latch_pin = GPIO_PIN_1,
			.oe_port = GPIOC,
			.oe_pin = GPIO_PIN_13,
			.value = 0
	};
	bme_spi_intf_sr spi_interface_bme = {
			.sr_pin = 2,
			.spi = &hspi2,
			.sr = &shift_reg_
	};
	lsm_spi_intf_sr spi_interface_lsm = {
			.sr_pin = 4,
			.spi = &hspi2,
			.sr = &shift_reg_
	};

	// ОСНОВНАЯ ЧАСТЬ, САМИ ФУНКЦИИ ИНИЦИАЛИЗАЦИИ ПРИБОРОВ
	spi_interface_lsm.sr = &shift_reg_;
	spi_interface_bme.sr = &shift_reg_;
	shift_reg_init(&shift_reg_);
	shift_reg_write_8(&shift_reg_, 0xFF);
	bme_init_default_sr(&bme, &spi_interface_bme);
	lsmset_sr (&ctx, &spi_interface_lsm);
	float acc_g[3];
	float gyro_dps[3];
	float temperature_celsius_mag;

	while(1)
	{
		struct bme280_data comp_data = bme_read_data(&bme);
		lsmread(&ctx, &temperature_celsius_mag, &acc_g, &gyro_dps);

		printf("ax:%10lf    ay:%10lf    az:%10lf    press: %lf    temp: %lf\n", acc_g[0], acc_g[1], acc_g[2], comp_data.pressure, comp_data.temperature);
		HAL_Delay(150);
	}

	return 0;
}
