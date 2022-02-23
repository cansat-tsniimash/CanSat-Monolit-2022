#ifndef GPS_H_
#define GPS_H_

#include <stdint.h>


extern int pavel_gps_sost;
extern char pavel_gps_buffer[300];
extern int pavel_gps_carret;


int gps_parse(uint8_t byte);




#endif /* GPS_H_ */
