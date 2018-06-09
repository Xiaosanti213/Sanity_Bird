#ifndef SENSORS_H_
#define SENSORS_H_

void ACC_getADC ();
void Gyro_getADC ();
uint8_t Mag_getADC();
uint8_t Baro_update();



void initSensors();

void i2c_rep_start(uint8_t address);
void i2c_write(uint8_t data );
void i2c_stop(void);
void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val);
uint8_t i2c_readReg(uint8_t add, uint8_t reg);
uint8_t i2c_readAck();
uint8_t i2c_readNak();


#endif