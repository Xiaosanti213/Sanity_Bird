#ifndef GPS_H_
#define GPS_H_


void GPS_set_pids(void);
void GPS_NewData(void);
void GPS_set_next_wp(int32_t* lat, int32_t* lon);
void GPS_reset_nav(void);
void GPS_I2C_command(uint8_t command, uint8_t wp);
int32_t wrap_18000(int32_t ang);




#endif
