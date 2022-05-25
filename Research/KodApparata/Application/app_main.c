
#include <stm32f4xx_hal.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>


#include "gps.h"
#include "fatfs.h"
#include "LSM6DS3/DLSM.h"
#include "BME280/DriverForBME280.h"
#include "nRF24L01_PL/nrf24_defs.h"
#include "nRF24L01_PL/nrf24_upper_api.h"
#include "nRF24L01_PL/nrf24_lower_api_stm32.h"
#include "nRF24L01_PL/nrf24_lower_api.h"
#include "Photorezistor/photorezistor.h"
#include "ATGM336H/nmea_gps.h"
#include "LIS3MDL/DLIS3.h"
#include "1Wire_DS18B20/one_wire.h"



extern ADC_HandleTypeDef hadc1;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

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

typedef struct reg_param_t
{
	uint8_t addr;
	const char * name;
	uint8_t size;
} reg_param_t;

static reg_param_t reg_params[] = {
	{ NRF24_REGADDR_CONFIG, "NRF24_REGADDR_CONFIG",			1},
	{ NRF24_REGADDR_EN_AA, "NRF24_REGADDR_EN_AA",				1},
	{ NRF24_REGADDR_EN_RXADDR, "NRF24_REGADDR_EN_RXADDR",		1},
	{ NRF24_REGADDR_SETUP_AW, "NRF24_REGADDR_SETUP_AW",		1},
	{ NRF24_REGADDR_SETUP_RETR, "NRF24_REGADDR_SETUP_RETR",	1},
	{ NRF24_REGADDR_RF_CH, "NRF24_REGADDR_RF_CH",				1},
	{ NRF24_REGADDR_RF_SETUP, "NRF24_REGADDR_RF_SETUP",		1},
	{ NRF24_REGADDR_STATUS, "NRF24_REGADDR_STATUS",			1},
	{ NRF24_REGADDR_OBSERVE_TX, "NRF24_REGADDR_OBSERVE_TX",	1},
	{ NRF24_REGADDR_RPD, "NRF24_REGADDR_RPD",					1},
	{ NRF24_REGADDR_RX_ADDR_P0, "NRF24_REGADDR_RX_ADDR_P0",	5},
	{ NRF24_REGADDR_RX_ADDR_P1, "NRF24_REGADDR_RX_ADDR_P1",	5},
	{ NRF24_REGADDR_RX_ADDR_P2, "NRF24_REGADDR_RX_ADDR_P2",	1},
	{ NRF24_REGADDR_RX_ADDR_P3, "NRF24_REGADDR_RX_ADDR_P3",	1},
	{ NRF24_REGADDR_RX_ADDR_P4, "NRF24_REGADDR_RX_ADDR_P4",	1},
	{ NRF24_REGADDR_RX_ADDR_P5, "NRF24_REGADDR_RX_ADDR_P5",	1},
	{ NRF24_REGADDR_TX_ADDR, "NRF24_REGADDR_TX_ADDR",			5},
	{ NRF24_REGADDR_RX_PW_P0, "NRF24_REGADDR_RX_PW_P0",		1},
	{ NRF24_REGADDR_RX_PW_P1, "NRF24_REGADDR_RX_PW_P1",		1},
	{ NRF24_REGADDR_RX_PW_P2, "NRF24_REGADDR_RX_PW_P2",		1},
	{ NRF24_REGADDR_RX_PW_P3, "NRF24_REGADDR_RX_PW_P3",		1},
	{ NRF24_REGADDR_RX_PW_P4, "NRF24_REGADDR_RX_PW_P4",		1},
	{ NRF24_REGADDR_RX_PW_P5, "NRF24_REGADDR_RX_PW_P5",		1},
	{ NRF24_REGADDR_FIFO_STATUS, "NRF24_REGADDR_FIFO_STATUS", 1},
	{ NRF24_REGADDR_DYNPD, "NRF24_REGADDR_DYNPD",				1},
	{ NRF24_REGADDR_FEATURE, "NRF24_REGADDR_FEATURE",			1}
};
//static
void print_bits(uint8_t value, char * buffer)
{
	for (size_t i = 0; i < sizeof(value)*8; i++)
	{
		int bit = value & (0x01 << 7);
		sprintf(buffer, "%d", bit ? 1 : 0);
		value = value << 1;
		buffer += 1;
	}
}
static void print_register(uint8_t reg_addr, uint8_t * reg_data)
{
	reg_param_t * selected_param = NULL;
	for (size_t i = 0; i < sizeof(reg_params)/sizeof(reg_params[0]); i++)
	{
		if (reg_addr == reg_params[i].addr)
		{
			selected_param  = &reg_params[i];
			break;
		}
	}

	if (NULL == selected_param)
	{

		printf("invalid reg addr: %d\n", reg_addr);
		return;
	}

	const char * reg_name = selected_param->name;
	const size_t reg_size = selected_param->size;

	printf("reg %s (0x%02X) = ", reg_name, (int)reg_addr);
	if (1 == reg_size)
	{
		printf("0x%02X", reg_data[0]);
		char bits_buffer[10] = {0};
		print_bits(reg_data[0], bits_buffer);
		printf(" (0b%s)", bits_buffer);
	}
	else
	{
		printf("0x");
		for (size_t j = 0; j < reg_size; j++)
		{
			printf("%02X", reg_data[j]);
		}
	}
	printf("\n");
}
static void dump_registers(void *intf_ptr)
{
	const size_t regs_count = sizeof(reg_params)/sizeof(reg_params[0]);
	for (size_t i = 0 ; i < regs_count; i++)
	{
		uint8_t reg_addr = reg_params[i].addr;
		uint8_t reg_size = reg_params[i].size;

		uint8_t reg_data[5] = { 0 };
		nrf24_read_register(intf_ptr, reg_addr, reg_data, reg_size);

		print_register(reg_addr, reg_data);
	}
}

#pragma pack(push,1)
// Структура пакетов в радио
typedef struct
{
	uint8_t flag;

	int16_t LSM6DSL_accelerometr_x;
	int16_t LSM6DSL_accelerometr_y;
	int16_t LSM6DSL_accelerometr_z;
	int16_t LSM6DSL_gyroscope_x;
	int16_t LSM6DSL_gyroscope_y;
	int16_t LSM6DSL_gyroscope_z;
	int16_t LIS3MDL_magnitometr_x;
	int16_t LIS3MDL_magnitometr_y;
	int16_t LIS3MDL_magnitometr_z;
	uint16_t num;
	uint16_t crc;

	uint32_t time;
	float lux;
}packet_orient;
typedef struct
{
	uint8_t flag;

	int16_t BME280_temperature;

	uint16_t num;
	uint16_t crc;

	uint32_t BME280_pressure;
	uint32_t time;
}packet_BME280;
typedef struct
{
	uint8_t GPS_fix;
	uint8_t flag;
	uint8_t state_apparate;

	int16_t GPS_altitude;
	uint16_t num;
	uint16_t crc;
	uint16_t DS18B20_temperature;

	float GPS_latitude;
	float GPS_longtitude;
	uint32_t time;
	uint32_t GPS_time_s;
	uint32_t GPS_time_us;
}packet_no_name;
typedef struct
{
	uint8_t flag;

	uint16_t num;
	uint16_t crc;

	uint32_t time;
	uint32_t tick_now;
	uint32_t tick_sum;
}packet_dosimetr;

#pragma pack(pop)

typedef enum {
	STATUS_PREROCKET,
	STATUS_START_DELAY,
	STATUS_IN_ROCKET,
	STATUS_FREE_FALL,
	STATUS_WATERING,
	STATUS_GROUND
} status_apparate_t;



int app_main(void)
{
			status_apparate_t status_apparate = STATUS_PREROCKET;
            // Карта памяти (ПЕРЕМЕННЫЕ)
			FATFS fileSystem; // переменная типа FATFS
			FIL SDFile; // хендлер файла
			UINT CheckBytes; // количество символов, реально записанных внутрь файла
			FRESULT resSD; // результат выполнения функции
			uint8_t path[11] = "SDcard.csv";
			path[10] =  '\0';

			// Переменные для работы с бме и лсм
			struct bme280_dev bme = {0};
			stmdev_ctx_t ctx = {0};


// НАСТРОЙКА РАДИО (СТРАКТЫ)
			shift_reg_t shift_reg_ = {
					.bus = &hspi2,
					.latch_port= GPIOC,
					.latch_pin = GPIO_PIN_1,
					.oe_port = GPIOC,
					.oe_pin = GPIO_PIN_13,
					.value = 0
			};
			shift_reg_init(&shift_reg_);
			shift_reg_oe(&shift_reg_, true);
			shift_reg_write_8(&shift_reg_, 0xFF);

			//Сдвиговый регистр радио
			shift_reg_t shift_reg_rf = {
					.bus = &hspi2,
					.latch_port = GPIOC,
					.latch_pin = GPIO_PIN_4,
					.oe_port = GPIOC,
					.oe_pin = GPIO_PIN_5,
					.value = 0
			};
			shift_reg_init(&shift_reg_rf);
			shift_reg_oe(&shift_reg_rf, true);
			shift_reg_write_8(&shift_reg_rf, 0xFF);
			shift_reg_oe(&shift_reg_rf, false);

			// Структура, содержащая параметры SPI пинов  Chip Enab и SPI Chip Select для сдвигового регистра
			nrf24_spi_pins_sr_t nrf24_spi_pins_sr = {
					.pos_CE = 0,
					.pos_CS = 1,
					.this = &shift_reg_rf
			};
			nrf24_lower_api_config_t nrf24_lower_api_config;
			nrf24_spi_init_sr(&nrf24_lower_api_config, &hspi2, &nrf24_spi_pins_sr);
			nrf24_mode_power_down(&nrf24_lower_api_config);



			// Настройка радиомодуля
			nrf24_rf_config_t rf_cfg_radio = {
					.data_rate = NRF24_DATARATE_250_KBIT,
					.tx_power = NRF24_TXPOWER_MINUS_0_DBM,
					.rf_channel = 111
			};
			nrf24_setup_rf(&nrf24_lower_api_config, &rf_cfg_radio);
			// Настройка протокола
			nrf24_protocol_config_t protocol_cfg_radio = {
					.crc_size = NRF24_CRCSIZE_1BYTE,
					.address_width = NRF24_ADDRES_WIDTH_5_BYTES,
					.en_dyn_payload_size = true,
					.en_ack_payload = true,
					.en_dyn_ack = true,
					.auto_retransmit_count = 0,
					.auto_retransmit_delay = 0
			};
			nrf24_pipe_config_t pipe_config = {
				.enable_auto_ack = true,
				.address = 0xdadadadada,
				.payload_size=-1
			};
			nrf24_setup_protocol(&nrf24_lower_api_config, &protocol_cfg_radio);

			nrf24_pipe_rx_start(&nrf24_lower_api_config, 0, &pipe_config);
			nrf24_pipe_rx_start(&nrf24_lower_api_config, 1, &pipe_config);
			nrf24_pipe_set_tx_addr(&nrf24_lower_api_config, 0xacacacacac);

// НАСТРОЙКА ЛСМ И БМЕ, Фоторезистор, ДС18 (СТРАКТЫ)

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
			photorezistor_t photorez_set = {
				.resist = 5100,
				.hadc = &hadc1
			};
			lis_spi_intf_sr spi_interface_lis = {
					.sr_pin = 3,
					.spi = &hspi2,
					.sr = &shift_reg_
			};
			ds18b20_resulution_t resulution_ds18b20 = {
					DS18B20_RESOLUTION_12_BIT
			};
			ds18b20_t onewire_intf_ds18b20 = {
					.onewire_port = GPIOA,
					.onewire_pin = GPIO_PIN_1
			};

	// ОСНОВНАЯ ЧАСТЬ, САМИ ФУНКЦИИ ИНИЦИАЛИЗАЦИИ ПРИБОРОВ
			resSD = f_mount(&fileSystem, "", 1);
			if(resSD != FR_OK) {
				printf("mount error, %d\n", resSD);
			}
			f_open(&SDFile, (char*)path, FA_WRITE | FA_CREATE_ALWAYS);


			nrf24_mode_standby(&nrf24_lower_api_config);
			//nrf24_mode_tx(&nrf24_lower_api_config);
			//dump_registers(&nrf24_lower_api_config);


			spi_interface_lsm.sr = &shift_reg_;
			spi_interface_bme.sr = &shift_reg_;
			spi_interface_lis.sr = &shift_reg_;

			gps_init();
			__HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);

			onewire_init(&onewire_intf_ds18b20);

			bme_init_default_sr(&bme, &spi_interface_bme);
			lsmset_sr(&ctx, &spi_interface_lsm);
			lisset_sr(&ctx, &spi_interface_lis);
			int8_t alarm_th = 50;
			int8_t alarm_tl = -50;
			ds18b20_set_config(&onewire_intf_ds18b20, alarm_th, alarm_tl, resulution_ds18b20);
			int64_t cookie = 0;
			uint64_t time_s = 0;
			uint32_t time_us = 0;
			uint16_t ds18_raw_temperature = 0;
			uint32_t start_time = 0;
			uint32_t watering_wait_time = 0;
			uint32_t height_BME = 0;
			uint32_t pressure_outdoor = 0;
			float lux = 0;
			float lux_outdoor = 0;
			int fix = 0;
			float lat = 0;
			float lon = 0;
			float alt = 0;
			float acc_g[3] = {0};
			float gyro_dps[3] = {0};
			float mag_raw[3] = {0};
			float temperature_celsius_mag = 0;
			char headbuffer[1000];
			int headcount = snprintf(headbuffer, 1000, "ax;ay;az;gx;gy;gz;magx;magy;magz;press;tempBME;lux;lat;lon;alt;cookie;time_s;time_us;fix;tempDS\n");
			f_write(&SDFile, (uint8_t*) headbuffer, headcount, &CheckBytes);
			f_sync(&SDFile);
			uint16_t packet_num_1 = 0;
			uint16_t packet_num_2 = 0;
			uint16_t packet_num_3 = 0;
			uint16_t packet_num_4 = 0;
			while(1)
			{
				HAL_ADC_Start(&hadc1);
				nrf24_fifo_status_t Status_FIFO_RX;
				nrf24_fifo_status_t Status_FIFO_TX;
				struct bme280_data comp_data = bme_read_data(&bme);
				lux = photorezistor_get_lux(photorez_set);
				lsmread(&ctx, &temperature_celsius_mag, &acc_g, &gyro_dps);
				lisread(&ctx, &temperature_celsius_mag, &mag_raw);
				gps_get_coords(&cookie,  &lat, &lon, &alt, &fix);
				gps_get_time(&cookie, &time_s, &time_us);
				ds18b20_start_conversion(&onewire_intf_ds18b20);
				ds18b20_read_raw_temperature(&onewire_intf_ds18b20, &ds18_raw_temperature, 0);
				float ds18_temp_celsius = ds18_raw_temperature / 16.0;

				// СОСТОЯНИЯ
				if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 1) && (status_apparate = STATUS_PREROCKET)) {
						status_apparate = STATUS_START_DELAY;
						start_time = HAL_GetTick();
						int count = 10;
						for (int i = 0; i < count; i++) {
							lux_outdoor += photorezistor_get_lux(photorez_set);
							pressure_outdoor += (uint32_t)comp_data.pressure;
						}
						lux_outdoor /= count;
						pressure_outdoor /= count;
				}
				height_BME = 44330*(1 - pow((float)comp_data.pressure/pressure_outdoor, 1.0/5.255));
				if ((HAL_GetTick() - start_time > 20000) && (status_apparate == STATUS_START_DELAY)) {
					status_apparate = STATUS_IN_ROCKET;
				}
				if ((lux  > 0.9 * lux_outdoor) && (status_apparate == STATUS_IN_ROCKET)) {
					status_apparate = STATUS_FREE_FALL;
					watering_wait_time = HAL_GetTick();
				}
				if ((height_BME < 300) && (status_apparate == STATUS_FREE_FALL) && (HAL_GetTick() - watering_wait_time > 5000)) {
					status_apparate = STATUS_WATERING;
					// рубить пин гпио отвеч-ий за компрессор и двигатель
				}




				packet_orient packet1 = {0};
				packet_BME280 packet2 = {0};
				packet_no_name packet3 = {0};
				packet_dosimetr packet4 = {0};

				packet1.flag = 228;
				packet1.LSM6DSL_accelerometr_x = (int16_t)(1000 * acc_g[0]);
				packet1.LSM6DSL_accelerometr_y = (int16_t)(1000 * acc_g[1]);
				packet1.LSM6DSL_accelerometr_z = (int16_t)(1000 * acc_g[2]);
				packet1.LSM6DSL_gyroscope_x = (int16_t)(100 * gyro_dps[0]);
				packet1.LSM6DSL_gyroscope_y = (int16_t)(100 * gyro_dps[1]);
				packet1.LSM6DSL_gyroscope_z = (int16_t)(100 * gyro_dps[2]);
				packet1.LIS3MDL_magnitometr_x = (int16_t)(1000 * mag_raw[0]);
				packet1.LIS3MDL_magnitometr_y = (int16_t)(1000 * mag_raw[1]);
				packet1.LIS3MDL_magnitometr_z = (int16_t)(1000 * mag_raw[2]);
				packet1.num = packet_num_1++;
				packet1.time = HAL_GetTick();
				packet1.crc = Crc16((uint8_t*) &packet1, sizeof(packet1));
				packet2.flag = 117;
				packet2.num = packet_num_2++;
				packet2.BME280_temperature = (int16_t)(10 * comp_data.temperature);
				packet2.BME280_pressure = (uint32_t)comp_data.pressure;
				packet2.crc = Crc16((uint8_t*) &packet2, sizeof(packet2));
				packet2.time = HAL_GetTick();
				packet3.flag = 99;
				packet3.num = packet_num_3++;
				packet3.GPS_altitude = (int16_t)(10 * alt);
				packet3.GPS_fix = (uint8_t)fix;
				packet3.GPS_latitude = lat;
				packet3.GPS_longtitude = lon;
				packet3.GPS_time_s = (uint32_t)time_s;
				packet3.GPS_time_us = time_us;
				packet3.DS18B20_temperature = ds18_temp_celsius;
				packet3.state_apparate = status_apparate;
				packet3.crc = Crc16((uint8_t*) &packet3, sizeof(packet3));
				packet3.time = HAL_GetTick();
				packet4.flag = 66;
				packet4.num = packet_num_4++;
				//packet4.tick_now =
				//packet4.tick_sum =
				packet4.crc = Crc16((uint8_t*) &packet4, sizeof(packet4));
				packet4.time = HAL_GetTick();


				char snbuffer[1000];
				int count = snprintf(snbuffer, 1000, "%10lf;%10lf;%10lf;%10lf;%10lf;%10lf;%10lf;%10lf;%10lf;%lf;%lf;%lf;%lf;%lf;%lf;%lld;%lld;%ld;%d;%lf\n", acc_g[0], acc_g[1], acc_g[2], gyro_dps[0], gyro_dps[1], gyro_dps[2], mag_raw[0], mag_raw[1], mag_raw[2], comp_data.pressure, comp_data.temperature, lux, lat, lon, alt, cookie, time_s, time_us, fix, ds18_temp_celsius);
				f_write(&SDFile, (uint8_t*) snbuffer, count, &CheckBytes);
				f_sync(&SDFile);


				//dump_registers(&nrf24_lower_api_config);
				nrf24_fifo_status(&nrf24_lower_api_config, &Status_FIFO_RX, &Status_FIFO_TX);
				if (Status_FIFO_TX != NRF24_FIFO_FULL) {
					nrf24_fifo_write(&nrf24_lower_api_config, (uint8_t*) &packet1, sizeof(packet1), false);
					nrf24_fifo_write(&nrf24_lower_api_config, (uint8_t*) &packet2, sizeof(packet2), false);
					nrf24_fifo_write(&nrf24_lower_api_config, (uint8_t*) &packet3, sizeof(packet3), false);
					nrf24_fifo_write(&nrf24_lower_api_config, (uint8_t*) &packet4, sizeof(packet4), false);
					nrf24_mode_tx(&nrf24_lower_api_config);
					HAL_Delay(5);
					nrf24_mode_standby(&nrf24_lower_api_config);

				}
				else
				{
					HAL_Delay(100);
					nrf24_fifo_flush_tx(&nrf24_lower_api_config);
				}

				nrf24_irq_clear(&nrf24_lower_api_config, NRF24_IRQ_RX_DR | NRF24_IRQ_TX_DR | NRF24_IRQ_MAX_RT);
				//HAL_UART_Transmit(&huart1, (uint8_t*) &packet, sizeof(packet), 100);
				//printf("lat: %lf, lon: %lf, alt: %lf, time_s: %ld, time_us: %d\n", (float) lat, (float) lon, (float) alt, (uint32_t) time_s, (int) time_us);
				//printf("tempDS: %lf\n", ds18_temp_celsius);
			}

	return 0;
}
