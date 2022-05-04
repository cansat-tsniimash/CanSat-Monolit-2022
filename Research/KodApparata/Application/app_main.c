
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
	uint8_t BME280_temperature;

	int16_t LSM6DSL_accelerometr_x;
	int16_t LSM6DSL_accelerometr_y;
	int16_t LSM6DSL_accelerometr_z;
	int16_t LSM6DSL_gyroscope_x;
	int16_t LSM6DSL_gyroscope_y;
	int16_t LSM6DSL_gyroscope_z;
	uint16_t num;
	uint16_t crc;

	uint32_t time;
	uint32_t BME280_pressure;
}packet_da_type_1_t;
#pragma pack(pop)

int app_main(void)
{

     /*       // Карта памяти (ПЕРЕМЕННЫЕ)
			FATFS fileSystem; // переменная типа FATFS
			FIL SDFile; // хендлер файла
			UINT CheckBytes; // количество символов, реально записанных внутрь файла
			FRESULT resSD; // результат выполнения функции
			uint8_t path[11] = "SDcard.csv";
			path[10] =  '\0';
	*/
			// Переменные для работы с бме и лсм
			struct bme280_dev bme = {0};
			stmdev_ctx_t ctx = {0};


// НАСТРОЙКА РАДИО (СТРАКТЫ)

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
			shift_reg_write_8(&shift_reg_rf, 0xFF);
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
			/*resSD = f_mount(&fileSystem, "", 1);
			if(resSD != FR_OK) {
				printf("mount error, %d\n", resSD);
			}
			f_open(&SDFile, (char*)path, FA_WRITE | FA_CREATE_ALWAYS);
	*/


			nrf24_mode_standby(&nrf24_lower_api_config);
			//nrf24_mode_tx(&nrf24_lower_api_config);


			dump_registers(&nrf24_lower_api_config);



			spi_interface_lsm.sr = &shift_reg_;
			spi_interface_bme.sr = &shift_reg_;
			shift_reg_init(&shift_reg_);
			shift_reg_write_8(&shift_reg_, 0xFF);
			bme_init_default_sr(&bme, &spi_interface_bme);
			lsmset_sr (&ctx, &spi_interface_lsm);
			float acc_g[3];
			float gyro_dps[3];
			float temperature_celsius_mag;
			/*char headbuffer[1000];
			int headcount = snprintf(headbuffer, 1000, "ax;ay;az;gx;gy;gz;temp;press;\n");
			f_write(&SDFile, (uint8_t*) headbuffer, headcount, &CheckBytes);
			f_sync(&SDFile);*/
			while(1)
			{
				nrf24_fifo_status_t Status_FIFO_RX;
				nrf24_fifo_status_t Status_FIFO_TX;
				struct bme280_data comp_data = bme_read_data(&bme);
				lsmread(&ctx, &temperature_celsius_mag, &acc_g, &gyro_dps);
				packet_da_type_1_t packet = {0};
				packet.flag = 228;
				packet.BME280_temperature = comp_data.temperature;
				packet.BME280_pressure = comp_data.pressure;
				packet.LSM6DSL_accelerometr_x = acc_g[0];
				packet.LSM6DSL_accelerometr_y = acc_g[1];
				packet.LSM6DSL_accelerometr_z = acc_g[2];
				packet.LSM6DSL_gyroscope_x = gyro_dps[0];
				packet.LSM6DSL_gyroscope_y = gyro_dps[1];
				packet.LSM6DSL_gyroscope_z = gyro_dps[2];
				packet.num = 0;
				packet.num++;
				packet.time = HAL_GetTick();
				packet.crc = Crc16((uint8_t*) &packet, sizeof(packet));

				/*char snbuffer[1000];
				int count = snprintf(snbuffer, 1000, "%10lf;%10lf;%10lf;%10lf;%10lf;%10lf;%lf;%lf;\n", acc_g[0], acc_g[1], acc_g[2], gyro_dps[0], gyro_dps[1], gyro_dps[2], comp_data.pressure, comp_data.temperature);
				f_write(&SDFile, (uint8_t*) snbuffer, count, &CheckBytes);
				f_sync(&SDFile);
				*/

				dump_registers(&nrf24_lower_api_config);
				nrf24_fifo_status(&nrf24_lower_api_config, &Status_FIFO_RX, &Status_FIFO_TX);

				if (Status_FIFO_TX != NRF24_FIFO_FULL) {
					nrf24_fifo_write(&nrf24_lower_api_config, (uint8_t*) &packet , sizeof(packet), false);

					nrf24_fifo_status(&nrf24_lower_api_config, &Status_FIFO_RX, &Status_FIFO_TX);

					nrf24_mode_tx(&nrf24_lower_api_config);
					HAL_Delay(100);
					nrf24_mode_standby(&nrf24_lower_api_config);
				}
				else
				{
					HAL_Delay(100);
					nrf24_fifo_flush_tx(&nrf24_lower_api_config);
				}

				nrf24_irq_clear(&nrf24_lower_api_config, NRF24_IRQ_RX_DR | NRF24_IRQ_TX_DR | NRF24_IRQ_MAX_RT);
				//HAL_UART_Transmit(&huart1, (uint8_t*) &packet, sizeof(packet), 100);
				//printf("ax: %10lf ay: %10lf az: %10lf gx: %10lf gy: %10lf gz: %10lf\n", acc_g[0], acc_g[1], acc_g[2], gyro_dps[0], gyro_dps[1], gyro_dps[2]);
				//printf("%lf %lf\n", comp_data.pressure, comp_data.temperature);
			}

	return 0;
}
