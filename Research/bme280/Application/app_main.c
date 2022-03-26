#include <stdio.h>

#include <stm32f4xx_hal.h>

#include "lsm6ds3_reg.h"

#include "DBME280.h"





int app_main(void)
{
	// Настройка bme280 =-=-=-=-=-=-=-=-=-=-=-=-
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	struct bme280_dev bme = {0};
	bmeset(&bme);
	while(1){
		struct bme280_data comp_data = bmewritt(&bme);
		printf("%lf\n", comp_data.temperature);
	}

	return 0;
}
