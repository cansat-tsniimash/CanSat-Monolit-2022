
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
#include "Dosimeter/dosimeter.h"

#define HEIGHT_WATERING 300
#define WATERING_DELAY 5000
#define STARTING_DELAY 20000 // ЗАМЕНИТЬ!!!!!!!!11!1
#define DELAY_FOR_WATERING 20000
#define SLEEPING_GROUND_HEIGHT_MEASURE_TIME 10000
#define COOLDOWN_NRF_WRITE 500
#define COOLDOWN_NRF_SPEED_WRITE 0 // TESTED
#define COOLDOWN_FSYNC_SD 1000


extern ADC_HandleTypeDef hadc1;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

//радио: настройка радио-части(+) и настройка протокольной части(+), настройка пайпа(+), перегон в режим отправки (тх) (+); потом радио пакты пихаем в ФИФО(проверка на свободное место в ФИФО),
//Переменные для ф-ии опр. приземления
int ground_buffer_num = 0;
uint32_t time_height_ground_measure = 0;
uint32_t buffer_ground_height[2] = {0};


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

//ФУНКЦИЯ ПРОВЕРКИ ПОСАДКИ АППАРАТА
static int check_for_landing(uint32_t height_BME) {
	if (HAL_GetTick() - time_height_ground_measure > SLEEPING_GROUND_HEIGHT_MEASURE_TIME) {
		buffer_ground_height[ground_buffer_num] = height_BME;
		ground_buffer_num += 1;
		time_height_ground_measure = HAL_GetTick();
		if (ground_buffer_num > 1) {
			ground_buffer_num = 0;
		}
	}
	if (abs(buffer_ground_height[1] - buffer_ground_height[0]) < 5) {
		return 1;
	}
	else {
		return 0;
	}
}

//ФУНКЦИЯ ДЛЯ ПЕРЕСЧЁТА ДАВЛЕНИЯ В ВЫСОТУ С БМЕ
static uint32_t count_height_BME(float pressure_BME, float pressure_outdoor) {
	uint32_t height_for_BME = 0;
	height_for_BME = 44330*(1 - pow(pressure_BME/pressure_outdoor, 1.0/5.255));
	return height_for_BME;
}

//ФУНКЦИИ ОТПРАВКИ ПАКЕТОВ ПО РАДИО С УЧЁТОМ НЕОБХОДИМОЙ ЗАДЕРЖКИ
static void fifo_write_packet_orient(void * intf_ptr, const uint8_t * packet, uint8_t packet_size, bool use_ack) {
	static uint32_t time_fifo_send = 0;
	if (HAL_GetTick() - time_fifo_send > COOLDOWN_NRF_SPEED_WRITE) {
		nrf24_fifo_write(intf_ptr, packet, packet_size, use_ack);
		time_fifo_send = HAL_GetTick();
	}
}
static void fifo_write_packet_BME280(void * intf_ptr, const uint8_t * packet, uint8_t packet_size, bool use_ack) {
	static uint32_t time_fifo_send = 0;
	if (HAL_GetTick() - time_fifo_send > COOLDOWN_NRF_SPEED_WRITE) {
		nrf24_fifo_write(intf_ptr, packet, packet_size, use_ack);
		time_fifo_send = HAL_GetTick();
	}
}
static void fifo_write_packet_ds_and_state(void * intf_ptr, const uint8_t * packet, uint8_t packet_size, bool use_ack) {
	static uint32_t time_fifo_send = 0;
	if (HAL_GetTick() - time_fifo_send > COOLDOWN_NRF_WRITE) {
		nrf24_fifo_write(intf_ptr, packet, packet_size, use_ack);
		time_fifo_send = HAL_GetTick();
	}

}

static void fifo_write_packet_dosimeter(void * intf_ptr, const uint8_t * packet, uint8_t packet_size, bool use_ack) {
	static uint32_t time_fifo_send = 0;
	if (HAL_GetTick() - time_fifo_send > COOLDOWN_NRF_WRITE) {
			nrf24_fifo_write(intf_ptr, packet, packet_size, use_ack);
			time_fifo_send = HAL_GetTick();
	}
}

static void fifo_write_packet_GPS(void * intf_ptr, const uint8_t * packet, uint8_t packet_size, bool use_ack) {
	static uint32_t time_fifo_send = 0;
	if (HAL_GetTick() - time_fifo_send > COOLDOWN_NRF_WRITE) {
			nrf24_fifo_write(intf_ptr, packet, packet_size, use_ack);
			time_fifo_send = HAL_GetTick();
	}
}


//ФУНКЦИЯ ПЕРЕМЕН МОДОВ ОТПРАВКИ РАДИО ДЛЯ ОТПРАВКИ ВСЕХ ТИПОВ ПАКЕТОВ
static void nrf_modes_plus_delay(void * intf_ptr) {
	nrf24_mode_tx(intf_ptr);
	HAL_Delay(3);
	nrf24_mode_standby(intf_ptr);
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

	uint32_t time;
	float lux;
	uint16_t crc;
}packet_orient_t;
typedef struct
{
	uint8_t flag;

	int16_t BME280_temperature;

	uint16_t num;

	uint32_t BME280_pressure;
	uint32_t time;

	uint16_t crc;
}packet_BME280_t;
typedef struct
{
	uint8_t flag;
	uint8_t state_apparate;

	uint16_t num;
	uint16_t DS18B20_temperature;

	uint32_t time;
	uint16_t crc;
}packet_ds_and_state_t;
typedef struct
{
	uint8_t flag;

	uint16_t num;

	uint32_t time;
	uint32_t tick_now;
	uint32_t tick_min;
	uint32_t tick_sum;

	uint16_t crc;
}packet_dosimetr_t;
typedef struct {
	uint8_t flag;
	uint8_t GPS_fix;

	int16_t GPS_altitude;
	uint16_t num;

	float GPS_latitude;
	float GPS_longtitude;
	uint32_t time;
	uint32_t GPS_time_s;
	uint32_t GPS_time_us;

	uint16_t crc;
}packet_GPS_t;

#pragma pack(pop)

typedef enum {
	STATUS_CALCULATED_PRESS_LUX,
	STATUS_PREROCKET,
	STATUS_START_DELAY,
	STATUS_IN_ROCKET,
	STATUS_FREE_FALL,
	STATUS_WATERING,
	STATUS_FREE_FALL_AFTER_WATERING,
	STATUS_GROUND
} status_apparate_t;



int app_main(void)
{



			// TESTED

			/*
			HAL_GPIO_WritePin(ENGINE_GPIO_Port, ENGINE_Pin, 1);
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
			HAL_Delay(7000);
			HAL_GPIO_WritePin(COMPRESSOR_GPIO_Port, COMPRESSOR_Pin, 1);
			*/





			status_apparate_t status_apparate = STATUS_CALCULATED_PRESS_LUX;
            // Карта памяти (ПЕРЕМЕННЫЕ)
			FATFS fileSystem; // переменная типа FATFS
			FIL SDFile; // хендлер файла
			UINT CheckBytes; // количество символов, реально записанных внутрь файла
			FRESULT resSD; // результат выполнения функции
			uint8_t path[11] = "SDcard.csv";
			path[10] =  '\0';
			char namebuffer[100];
			uint32_t namecount = 0;

			// Переменные для работы с бме и лсм
			struct bme280_dev bme = {0};
			stmdev_ctx_t ctx_lsm = {0};
			stmdev_ctx_t ctx_lis = {0};


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
				.resist = 2000,
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
			snprintf(namebuffer, 100, "AttemptFile%ld.csv", namecount);
			namecount++;
			f_open(&SDFile, namebuffer, FA_WRITE | FA_CREATE_NEW);


			nrf24_mode_standby(&nrf24_lower_api_config);
			//nrf24_mode_tx(&nrf24_lower_api_config);
			//dump_registers(&nrf24_lower_api_config);

			// ИНИЦИАЛИЗАЦИЯ ВСЕХ ШИФТ-РЕГИСТРОВ ДЛЯ ПРИБОРОВ
			spi_interface_lsm.sr = &shift_reg_;
			spi_interface_bme.sr = &shift_reg_;
			spi_interface_lis.sr = &shift_reg_;

			//ИНИЦИАЛИЗАЦИЯ ГПС-КИ И ВКЛЮЧЕНИЕ УАРТА
			gps_init();
			__HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);

			// ИНИЦИАЛИЗАЦИЯ ВАН-ВАЙРА
			onewire_init(&onewire_intf_ds18b20);


			//ИНИЦИАЛИЗАЦИЯ БМЕШКИ
			bme_init_default_sr(&bme, &spi_interface_bme);

			//ИНИЦИАЛИЗАЦИЯ ЛСМ И ЛИСА
			lsmset_sr(&ctx_lsm, &spi_interface_lsm);
			lisset_sr(&ctx_lis, &spi_interface_lis);

			// ВВОД ПЕРЕМЕННЫХ И ИНИЦИАЛИЗАЦИЯ ДС18Б20
			int8_t alarm_th = 50;
			int8_t alarm_tl = -50;
			ds18b20_set_config(&onewire_intf_ds18b20, alarm_th, alarm_tl, resulution_ds18b20);
			uint16_t ds18_raw_temperature = 0;

			// ПЕРЕМЕННЫЕ ДЛЯ СОСТОЯНИЙ АППАРАТА
			uint32_t start_time = 0;
			uint32_t watering_wait_time = 0;
			uint32_t time_of_watering = 0;
			uint32_t height_BME = 0;
			float pressure_outdoor = 0;

			//ПЕРЕМЕННЫЕ ДЛЯ ФОТОРЕЗИСТОРА
			float lux = 0;
			float lux_outdoor = 0;

			//ПЕРЕМЕННЫЕ ДЛЯ ГПС-КИ
			int fix = 0;
			int64_t cookie = 0;
			uint64_t time_s = 0;
			uint32_t time_us = 0;
			float lat = 0;
			float lon = 0;
			float alt = 0;

			//ПЕРЕМЕННЫЕ ДЛЯ АКСЕЛЕРОМЕТРА, ГИРОСКОПА И МАГНИТОМЕТРА СООТВЕТСТВЕННО
			float acc_g[3] = {0};
			float gyro_dps[3] = {0};
			float mag_raw[3] = {0};
			float temperature_celsius_mag = 0;

			//ПЕРЕМЕННЫЕ ДЛЯ ДОЗИМЕТРА
			uint32_t tick_sum = 0;
			uint32_t tick_now = 0;
			uint32_t tick_min = 0;


			//ЗАПИСЬ ЗАГОЛОВКОВ ТЕЛЕМЕТРИИ НА СД-КАРТУ
			uint32_t time_from_last_FSYNC = 0;
			char headbuffer[1000];
			int headcount = snprintf(headbuffer, 1000, "ax;ay;az;gx;gy;gz;magx;magy;magz;press;tempBME;lux;lat;lon;alt;cookie;time_s;time_us;fix;tempDS;state_apparat;tick_now;tick_min;tick_sum;pressure_out;lux_outdoor;height_BME\n");
			f_write(&SDFile, (uint8_t*) headbuffer, headcount, &CheckBytes);
			f_sync(&SDFile);

			//НУМЕРАЦИЯ ПАКЕТОВ РАДИО
			uint16_t packet_num_orient = 0;
			uint16_t packet_num_BME280 = 0;
			uint16_t packet_num_ds_state = 0;
			uint16_t packet_num_dosimetr = 0;
			uint16_t packet_num_GPS = 0;



			while(1)
			{
				//ЗАПУСК АЦП
				//uint32_t start = HAL_GetTick(); (TESTED)
				HAL_ADC_Start(&hadc1);

				//ОБЪЯВЛЕНИЕ ПЕРЕМЕННЫХ ДЛЯ ЗАПИСИ СОСТОЯНИЯ ФИФО-БУФФЕРА
				nrf24_fifo_status_t Status_FIFO_RX;
				nrf24_fifo_status_t Status_FIFO_TX;

				//ЗАПИСЬ ТЕЛЕМЕТРИИ С БМЕ
				struct bme280_data comp_data = bme_read_data(&bme);

				//ЗАПИСЬ ПОКАЗАНИЙ ФОТОРЕЗИСТОРА
				lux = photorezistor_get_lux(photorez_set);

				//ЧТЕНИЕ С ЛИС И ЛСМ
				lsmread(&ctx_lsm, &temperature_celsius_mag, &acc_g, &gyro_dps);
				lisread(&ctx_lis, &temperature_celsius_mag, &mag_raw);

				//ТЕЛЕМЕТРИЯ С ГПС-КИ
				gps_work();
				gps_get_coords(&cookie,  &lat, &lon, &alt, &fix);
				gps_get_time(&cookie, &time_s, &time_us);

				//ЧТЕНИЕ С ДОЗИМЕТРА
				tick_sum = Dosimeter_Get_Sum();
				tick_now = Dosimeter_Get_TPS();
				tick_min = Dosimeter_Get_TPM();

				//ЧТЕНИЕ И ПЕРЕВОД В ЧЕЛОВЕЧЕСКИЕ ВЕЛИЧИНЫ ПОКАЗАНИЙ С ДС18Б20
				ds18b20_start_conversion(&onewire_intf_ds18b20);
				ds18b20_read_raw_temperature(&onewire_intf_ds18b20, &ds18_raw_temperature, 0);
				float ds18_temp_celsius = ds18_raw_temperature / 16.0;

				// СОСТОЯНИЯ
				if (status_apparate == STATUS_CALCULATED_PRESS_LUX) {
					//ЗАМЕР СОСТОЯНИЯ ОКРУЖАЮЩЕЙ СРЕДЫ
					status_apparate = STATUS_PREROCKET;
					int count = 10;
					for (int i = 0; i < count; i++) {
						lux_outdoor += photorezistor_get_lux(photorez_set);
						pressure_outdoor += (uint32_t)comp_data.pressure;
					}
					lux_outdoor /= count;
					pressure_outdoor /= count;
				}
				if ((HAL_GPIO_ReadPin(SECOND_LEVER_GPIO_Port, SECOND_LEVER_Pin) != 0) && (status_apparate == STATUS_PREROCKET)) { // Условие на вкл 2ого тумблера (а4)
						status_apparate = STATUS_START_DELAY;
						start_time = HAL_GetTick();
				}
				height_BME = count_height_BME(comp_data.pressure, pressure_outdoor);
				if ((HAL_GetTick() - start_time > STARTING_DELAY) && (status_apparate == STATUS_START_DELAY)) {
					status_apparate = STATUS_IN_ROCKET;
				}
				if ((lux  > 0.9 * lux_outdoor) && (status_apparate == STATUS_IN_ROCKET)) {
					status_apparate = STATUS_FREE_FALL;
					watering_wait_time = HAL_GetTick();
					HAL_GPIO_WritePin(ENGINE_GPIO_Port, ENGINE_Pin, 1); // ВКЛ двигателя центрифуги-распрыскивателя
				}
				if ((height_BME < HEIGHT_WATERING) && (status_apparate == STATUS_FREE_FALL) && (HAL_GetTick() - watering_wait_time > WATERING_DELAY)) {
					status_apparate = STATUS_WATERING;
					HAL_GPIO_WritePin(COMPRESSOR_GPIO_Port, COMPRESSOR_Pin, 1); // ВКЛ компрессора
					time_of_watering = HAL_GetTick();
				}
				if ((HAL_GetTick() - time_of_watering > DELAY_FOR_WATERING) && (status_apparate == STATUS_WATERING)) {
					status_apparate = STATUS_FREE_FALL_AFTER_WATERING;
					HAL_GPIO_WritePin(COMPRESSOR_GPIO_Port, COMPRESSOR_Pin, 0); // ВЫКЛ компрессора
					HAL_GPIO_WritePin(ENGINE_GPIO_Port, ENGINE_Pin, 0); // ВЫКЛ двигателя центрифуги-распрыскивателя
				}
				if ((check_for_landing(height_BME)) && (status_apparate == STATUS_FREE_FALL_AFTER_WATERING)) {
					status_apparate = STATUS_GROUND;
					HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1); //ВКЛ пищалки
				}





				// ПАКЕТЫ РАДИО


				//ОБЪЯВЛЕНИЕ ПАКЕТОВ
				packet_orient_t packet_orient = {0};
				packet_BME280_t packet_BME280 = {0};
				packet_ds_and_state_t packet_ds_and_state = {0};
				packet_dosimetr_t packet_dosimetr = {0};
				packet_GPS_t packet_GPS = {0};

				//ЗАПОЛНЕНИЕ ПАКЕТОВ ТЕЛЕМЕТРИЕЙ
				packet_orient.flag = 228;
				packet_orient.LSM6DSL_accelerometr_x = (int16_t)(1000 * acc_g[0]);
				packet_orient.LSM6DSL_accelerometr_y = (int16_t)(1000 * acc_g[1]);
				packet_orient.LSM6DSL_accelerometr_z = (int16_t)(1000 * acc_g[2]);
				packet_orient.LSM6DSL_gyroscope_x = (int16_t)(100 * gyro_dps[0]);
				packet_orient.LSM6DSL_gyroscope_y = (int16_t)(100 * gyro_dps[1]);
				packet_orient.LSM6DSL_gyroscope_z = (int16_t)(100 * gyro_dps[2]);
				packet_orient.LIS3MDL_magnitometr_x = (int16_t)(1000 * mag_raw[0]);
				packet_orient.LIS3MDL_magnitometr_y = (int16_t)(1000 * mag_raw[1]);
				packet_orient.LIS3MDL_magnitometr_z = (int16_t)(1000 * mag_raw[2]);
				packet_orient.num = packet_num_orient++;
				packet_orient.time = HAL_GetTick();
				packet_orient.crc = Crc16((uint8_t*) &packet_orient, sizeof(packet_orient) - 2);
				packet_BME280.flag = 117;
				packet_BME280.num = packet_num_BME280++;
				packet_BME280.BME280_temperature = (int16_t)(10 * comp_data.temperature);
				packet_BME280.BME280_pressure = (uint32_t)comp_data.pressure;
				packet_BME280.time = HAL_GetTick();
				packet_BME280.crc = Crc16((uint8_t*) &packet_BME280, sizeof(packet_BME280));
				packet_ds_and_state.flag = 99;
				packet_ds_and_state.num = packet_num_ds_state++;
				packet_ds_and_state.DS18B20_temperature = ds18_temp_celsius;
				packet_ds_and_state.state_apparate = status_apparate;
				packet_ds_and_state.time = HAL_GetTick();
				packet_ds_and_state.crc = Crc16((uint8_t*) &packet_ds_and_state, sizeof(packet_ds_and_state) - 2);
				packet_dosimetr.flag = 66;
				packet_dosimetr.num = packet_num_dosimetr++;
				packet_dosimetr.tick_now = tick_now;
				packet_dosimetr.tick_min = tick_min;
				packet_dosimetr.tick_sum = tick_sum;
				packet_dosimetr.time = HAL_GetTick();
 				packet_dosimetr.crc = Crc16((uint8_t*) &packet_dosimetr, sizeof(packet_dosimetr) - 2);
				packet_GPS.flag = 71;
				packet_GPS.num = packet_num_GPS++;
				packet_GPS.GPS_altitude = (int16_t)(10 * alt);
				packet_GPS.GPS_fix = (uint8_t)fix;
				packet_GPS.GPS_latitude = lat;
				packet_GPS.GPS_longtitude = lon;
				packet_GPS.GPS_time_s = (uint32_t)time_s;
				packet_GPS.GPS_time_us = time_us;
				packet_GPS.time = HAL_GetTick();
				packet_GPS.crc = Crc16((uint8_t*) &packet_GPS, sizeof(packet_GPS) - 2);

				// ЗАПИСЬ НА СД-КАРТУ
				char snbuffer[1000];
				int count = snprintf(snbuffer, 1000, "%10lf;%10lf;%10lf;%10lf;%10lf;%10lf;%10lf;%10lf;%10lf;%lf;%lf;%lf;%lf;%lf;%lf;%lld;%lld;%ld;%d;%lf;%d;%ld;%ld;%ld;%lf;%lf;%ld\n", acc_g[0], acc_g[1], acc_g[2], gyro_dps[0], gyro_dps[1], gyro_dps[2], mag_raw[0], mag_raw[1], mag_raw[2], comp_data.pressure, comp_data.temperature,  lux, lat, lon, alt, cookie, time_s, time_us, fix, ds18_temp_celsius, status_apparate, tick_now, tick_min, tick_sum, pressure_outdoor, lux_outdoor, height_BME);
				FRESULT res = f_write(&SDFile, (uint8_t*) snbuffer, count, &CheckBytes);
				if (res != FR_OK) {
					f_close(&SDFile);
					snprintf(namebuffer, 100, "AttemptFile%ld.csv", namecount);
					namecount++;
					FRESULT resOPENSD = f_open(&SDFile, namebuffer, FA_WRITE | FA_CREATE_NEW);
						if ((resOPENSD != FR_OK) && (resOPENSD != FR_EXIST)) {
							f_mount(NULL, "", 1);
							FRESULT resMOUNTSD = f_mount(&fileSystem, "", 1);
							if (resMOUNTSD != FR_OK) {
								HAL_NVIC_SystemReset();
							}
						}
				}
				if (HAL_GetTick() - time_from_last_FSYNC > COOLDOWN_FSYNC_SD) {
					time_from_last_FSYNC = HAL_GetTick();
					f_sync(&SDFile);
				}
				//ОТПРАВКА ПАКЕТОВ ПО РАДИО
				//dump_registers(&nrf24_lower_api_config);
				nrf24_fifo_status(&nrf24_lower_api_config, &Status_FIFO_RX, &Status_FIFO_TX);
				if (Status_FIFO_TX != NRF24_FIFO_FULL) {
					fifo_write_packet_orient(&nrf24_lower_api_config, (uint8_t*) &packet_orient, sizeof(packet_orient), false);
					nrf_modes_plus_delay(&nrf24_lower_api_config);
					fifo_write_packet_BME280(&nrf24_lower_api_config, (uint8_t*) &packet_BME280, sizeof(packet_BME280), false);
					nrf_modes_plus_delay(&nrf24_lower_api_config);
					fifo_write_packet_ds_and_state(&nrf24_lower_api_config, (uint8_t*) &packet_ds_and_state, sizeof(packet_ds_and_state), false);
					nrf_modes_plus_delay(&nrf24_lower_api_config);
					fifo_write_packet_dosimeter(&nrf24_lower_api_config, (uint8_t*) &packet_dosimetr, sizeof(packet_dosimetr), false);
					nrf_modes_plus_delay(&nrf24_lower_api_config);
					fifo_write_packet_GPS(&nrf24_lower_api_config, (uint8_t*) &packet_GPS, sizeof(packet_GPS), false);
					nrf_modes_plus_delay(&nrf24_lower_api_config);
				}
				else
				{
					HAL_Delay(100);
					nrf24_fifo_flush_tx(&nrf24_lower_api_config);
				}

				//ЧИСТКА ФЛАГОВ РАДИО
				nrf24_irq_clear(&nrf24_lower_api_config, NRF24_IRQ_RX_DR | NRF24_IRQ_TX_DR | NRF24_IRQ_MAX_RT);
				//HAL_UART_Transmit(&huart1, (uint8_t*) &packet, sizeof(packet), 100);
				//printf("lat: %lf, lon: %lf, alt: %lf, time_s: %ld, time_us: %ld, fix: %d\n", (float) lat, (float) lon, (float) alt, (uint32_t) time_s, time_us, fix);
				//printf("tick_sum: %ld, tick_now: %ld, tick_min: %ld\n", tick_sum, tick_now, tick_min);
				//printf("pres:%lf, temp:%lf\n", comp_data.pressure, comp_data.temperature);
				//printf("ax: %10lf ay: %10lf az: %10lf gx: %10lf gy: %10lf gz: %10lf\n", acc_g[0], acc_g[1], acc_g[2], gyro_dps[0], gyro_dps[1], gyro_dps[2]);
				//printf("magx: %10lf, magy: %10lf, magz: %10lf\n", mag_raw[0], mag_raw[1], mag_raw[2]);
				//printf("lux: %f\n", lux);
				//printf("sost: %d\n", status_apparate);
				//TESTED:      uint32_t stop = HAL_GetTick();
				//printf("clock: %ld\n", start-stop);

			}

	return 0;
}
