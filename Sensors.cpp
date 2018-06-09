#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Alarms.h"
#include "EEPROM.h"
#include "IMU.h"


void i2c_BMP085_UT_Start(void);
void waitTransmissionI2C();
void i2c_MS561101BA_UT_Start();
void Baro_Common();
void Device_Mag_getADC();
void getADC();

void Baro_init();
void Mag_init();
void ACC_init();

//传感器取向配置，在config.h当中当前选择HK_MultiWii_SE_V2，对应在def.h中已经设置
/*
#define ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  = X; imu.accADC[PITCH]  = Y; imu.accADC[YAW]  = Z;}
#define GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[ROLL] = X; imu.gyroADC[PITCH] = Y; imu.gyroADC[YAW] = Z;}
#define MAG_ORIENTATION(X, Y, Z)  {imu.magADC[ROLL]  = X; imu.magADC[PITCH]  = Y; imu.magADC[YAW]  = Z;}
*/

#define MPU6050_ADDRESS     0x68


//当前配置文件中启用98Hz
#if defined(MPU6050_LPF_256HZ) || defined(MPU6050_LPF_188HZ) || defined(MPU6050_LPF_98HZ) || defined(MPU6050_LPF_42HZ) || defined(MPU6050_LPF_20HZ) || defined(MPU6050_LPF_10HZ) || defined(MPU6050_LPF_5HZ)
  #if defined(MPU6050_LPF_256HZ)
    #define MPU6050_DLPF_CFG   0
  #endif
  #if defined(MPU6050_LPF_188HZ)
    #define MPU6050_DLPF_CFG   1
  #endif
  #if defined(MPU6050_LPF_98HZ)
    #define MPU6050_DLPF_CFG   2
  #endif
  #if defined(MPU6050_LPF_42HZ)
    #define MPU6050_DLPF_CFG   3
  #endif
  #if defined(MPU6050_LPF_20HZ)
    #define MPU6050_DLPF_CFG   4
  #endif
  #if defined(MPU6050_LPF_10HZ)
    #define MPU6050_DLPF_CFG   5
  #endif
  #if defined(MPU6050_LPF_5HZ)
    #define MPU6050_DLPF_CFG   6
  #endif
#else
    //默认设置 LPF 256Hz/8000Hz 采样率
    #define MPU6050_DLPF_CFG   0
#endif



uint8_t rawADC[6];
static uint32_t neutralizeTime = 0;


//下面是i2c驱动
void i2c_init(void) {
	
  I2C_PULLUPS_DISABLE						   // 参考配置文件（使用外部上拉）不启用内部上拉
  TWSR = 0;                                    // no prescaler => prescaler = 1
  TWBR = ((F_CPU / I2C_SPEED) - 16) / 2;       // change the I2C clock rate
  TWCR = 1<<TWEN;                              // enable twi module, no interrupt
}


void i2c_rep_start(uint8_t address) {
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) ; // 发送起始条件
  waitTransmissionI2C();                       // 等到传输结束
  TWDR = address;                              // 发送从机设备地址
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();                       // 等到传输完成
}


void i2c_stop(void) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  //  while(TWCR & (1<<TWSTO));                // <- can produce a blocking state with some WMP clones
}


void i2c_write(uint8_t data ) {
  TWDR = data;                                 // 发送数据到指定地址了的设备
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();
}


uint8_t i2c_read(uint8_t ack) {
  TWCR = (1<<TWINT) | (1<<TWEN) | (ack? (1<<TWEA) : 0);
  waitTransmissionI2C();
  uint8_t r = TWDR;
  if (!ack) i2c_stop();
  return r;
}


uint8_t i2c_readAck() {
  return i2c_read(1);
}

uint8_t i2c_readNak(void) {
  return i2c_read(0);
}

void waitTransmissionI2C() {
  uint16_t count = 255;
  while (!(TWCR & (1<<TWINT))) {
    count--;
    if (count==0) {              //we are in a blocking state => we don't insist
      TWCR = 0;                  //and we force a reset on TWINT register
      neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay
      i2c_errors_count++;
      break;
    }
  }
}

size_t i2c_read_to_buf(uint8_t add, void *buf, size_t size) {
  i2c_rep_start((add<<1) | 1);   // I2C read direction
  size_t bytes_read = 0;
  uint8_t *b = (uint8_t*)buf;
  while (size--) {
    /* acknowledge all but the final byte */
    *b++ = i2c_read(size > 0);
    /* TODO catch I2C errors here and abort */
    bytes_read++;
  }
  return bytes_read;
}

size_t i2c_read_reg_to_buf(uint8_t add, uint8_t reg, void *buf, size_t size) {
  i2c_rep_start(add<<1); // I2C write direction
  i2c_write(reg);        // register selection
  return i2c_read_to_buf(add, buf, size);
}

/* 一系列字节大小端交换 */
void swap_endianness(void *buf, size_t size) {
  /* we swap in-place, so we only have to
  * place _one_ element on a temporary tray
  */
  uint8_t tray;
  uint8_t *from;
  uint8_t *to;
  /* keep swapping until the pointers have assed each other */
  for (from = (uint8_t*)buf, to = &from[size-1]; from < to; from++, to--) {
    tray = *from;
    *from = *to;
    *to = tray;
  }
}

void i2c_getSixRawADC(uint8_t add, uint8_t reg) {
  i2c_read_reg_to_buf(add, reg, &rawADC, 6);
}

void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val) {
  i2c_rep_start(add<<1); // I2C write direction
  i2c_write(reg);        // register selection
  i2c_write(val);        // value to write in register
  i2c_stop();
}

uint8_t i2c_readReg(uint8_t add, uint8_t reg) {
  uint8_t val;
  i2c_read_reg_to_buf(add, reg, &val, 1);
  return val;
}




static float   magGain[3] = {1.0,1.0,1.0};  // gain for each axis, populated at sensor init
static uint8_t magInit = 0;

uint8_t Mag_getADC() { // return 1 when news values are available, 0 otherwise
  static uint32_t t,tCal = 0;
  static int16_t magZeroTempMin[3];
  static int16_t magZeroTempMax[3];
  uint8_t axis;
  if ( currentTime < t ) return 0; //each read is spaced by 100ms
  t = currentTime + 100000;
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  Device_Mag_getADC();
  imu.magADC[ROLL]  = imu.magADC[ROLL]  * magGain[ROLL];
  imu.magADC[PITCH] = imu.magADC[PITCH] * magGain[PITCH];
  imu.magADC[YAW]   = imu.magADC[YAW]   * magGain[YAW];
  if (f.CALIBRATE_MAG) {
    tCal = t;
    for(axis=0;axis<3;axis++) {
      global_conf.magZero[axis] = 0;
      magZeroTempMin[axis] = imu.magADC[axis];
      magZeroTempMax[axis] = imu.magADC[axis];
    }
    f.CALIBRATE_MAG = 0;
  }
  if (magInit) { // we apply offset only once mag calibration is done
    imu.magADC[ROLL]  -= global_conf.magZero[ROLL];
    imu.magADC[PITCH] -= global_conf.magZero[PITCH];
    imu.magADC[YAW]   -= global_conf.magZero[YAW];
  }
 
  if (tCal != 0) {
    if ((t - tCal) < 30000000) { // 30s: you have 30s to turn the multi in all directions
      LEDPIN_TOGGLE;
      for(axis=0;axis<3;axis++) {
        if (imu.magADC[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = imu.magADC[axis];
        if (imu.magADC[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = imu.magADC[axis];
      }
    } else {
      tCal = 0;
      for(axis=0;axis<3;axis++)
        global_conf.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis])>>1;
      writeGlobalSet(1);
    }
  } else {
    #if defined(SENSORS_TILT_45DEG_LEFT)
      int16_t temp = ((imu.magADC[PITCH] - imu.magADC[ROLL] )*7)/10;
      imu.magADC[ROLL] = ((imu.magADC[ROLL]  + imu.magADC[PITCH])*7)/10;
      imu.magADC[PITCH] = temp;
    #endif
    #if defined(SENSORS_TILT_45DEG_RIGHT)
      int16_t temp = ((imu.magADC[PITCH] + imu.magADC[ROLL] )*7)/10;
      imu.magADC[ROLL] = ((imu.magADC[ROLL]  - imu.magADC[PITCH])*7)/10;
      imu.magADC[PITCH] = temp;
    #endif
  }
  return 1;
}










// 陀螺 加计 气压计通用函数
void GYRO_Common() {
  static int16_t previousGyroADC[3] = {0,0,0};
  static int32_t g[3];
  uint8_t axis, tilt=0;
  
  //只是校准了一次
  if (calibratingG>0) {
    for (axis = 0; axis < 3; axis++) {
      // 校准开始阶段，复位g[axis]的值
      if (calibratingG == 512) {
        g[axis]=0;
      }
      // 累加512次数据
      g[axis] +=imu.gyroADC[axis];
      // 清除全局变量，用以下一次读取
      imu.gyroADC[axis]=0;
      gyroZero[axis]=0;
      if (calibratingG == 1) {
        gyroZero[axis]=(g[axis]+256)>>9;
        blinkLED(10,15,1);
      }
    }
      calibratingG--;
  }
	//下面将原始数据-校准的零偏值
    for (axis = 0; axis < 3; axis++) {
    imu.gyroADC[axis]  -= gyroZero[axis];
    //防止陀螺仪出现偏差，限制两次连续读取的变量范围
    imu.gyroADC[axis] = constrain(imu.gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);

    previousGyroADC[axis] = imu.gyroADC[axis];
  }
  
}

  
  
void ACC_Common() {
  static int32_t a[3];
  if (calibratingA>0) {
    for (uint8_t axis = 0; axis < 3; axis++) {
      // 校准开始阶段，重置a[axis]
      if (calibratingA == 512) a[axis]=0;
      // 512次读取加和
      a[axis] +=imu.accADC[axis];
      // 清除全局变量以便下一次读取
      imu.accADC[axis]=0;
      global_conf.accZero[axis]=0;
    }
    // 计算均值，向下平移z轴ACC_1G，然后在校准过程结尾将值存储到EEPROM当中
    if (calibratingA == 1) {
      global_conf.accZero[ROLL]  = (a[ROLL]+256)>>9;
      global_conf.accZero[PITCH] = (a[PITCH]+256)>>9;
      global_conf.accZero[YAW]   = ((a[YAW]+256)>>9)-ACC_1G; // for nunchuk 200=1G
      conf.angleTrim[ROLL]   = 0;
      conf.angleTrim[PITCH]  = 0;
      writeGlobalSet(1); // 将结果写入EEPROM
    }
    calibratingA--;
  }
  
  imu.accADC[ROLL]  -=  global_conf.accZero[ROLL] ;
  imu.accADC[PITCH] -=  global_conf.accZero[PITCH];
  imu.accADC[YAW]   -=  global_conf.accZero[YAW] ;
 }
 

// BMP085驱动
#define BMP085_ADDRESS 0x77
static struct {
  // BOSCH BMP085 数据手册上的寄存器
  int16_t  ac1, ac2, ac3;
  uint16_t ac4, ac5, ac6;
  int16_t  b1, b2, mb, mc, md;
  union {uint16_t val; uint8_t raw[2]; } ut; //uncompensated T
  union {uint32_t val; uint8_t raw[4]; } up; //uncompensated P
  uint8_t  state;
  uint32_t deadline;
} bmp085_ctx; //已经实例化
 
 
#define OSS 3
void i2c_BMP085_readCalibration(){
  delay(10);
  //read calibration data in one go
  size_t s_bytes = (uint8_t*)&bmp085_ctx.md - (uint8_t*)&bmp085_ctx.ac1 + sizeof(bmp085_ctx.ac1);
  i2c_read_reg_to_buf(BMP085_ADDRESS, 0xAA, &bmp085_ctx.ac1, s_bytes);
  // now fix endianness
  int16_t *p;
  for (p = &bmp085_ctx.ac1; p <= &bmp085_ctx.md; p++) {
    swap_endianness(p, sizeof(*p));
  }
}

void  Baro_init() {
  delay(10);
  i2c_BMP085_readCalibration();
  delay(5);
  i2c_BMP085_UT_Start(); 
  bmp085_ctx.deadline = currentTime+5000;
}

// read uncompensated temperature value: send command first
void i2c_BMP085_UT_Start(void) {
  i2c_writeReg(BMP085_ADDRESS,0xf4,0x2e);
  i2c_rep_start(BMP085_ADDRESS<<1);
  i2c_write(0xF6);
  i2c_stop();
}

// read uncompensated pressure value: send command first
void i2c_BMP085_UP_Start () {
  i2c_writeReg(BMP085_ADDRESS,0xf4,0x34+(OSS<<6)); // control register value for oversampling setting 3
  i2c_rep_start(BMP085_ADDRESS<<1); //I2C write direction => 0
  i2c_write(0xF6);
  i2c_stop();
}

// read uncompensated pressure value: read result bytes
// the datasheet suggests a delay of 25.5 ms (oversampling settings 3) after the send command
void i2c_BMP085_UP_Read () {
  i2c_rep_start((BMP085_ADDRESS<<1) | 1);//I2C read direction => 1
  bmp085_ctx.up.raw[2] = i2c_readAck();
  bmp085_ctx.up.raw[1] = i2c_readAck();
  bmp085_ctx.up.raw[0] = i2c_readNak();
}

// read uncompensated temperature value: read result bytes
// the datasheet suggests a delay of 4.5 ms after the send command
void i2c_BMP085_UT_Read() {
  i2c_rep_start((BMP085_ADDRESS<<1) | 1);//I2C read direction => 1
  bmp085_ctx.ut.raw[1] = i2c_readAck();
  bmp085_ctx.ut.raw[0] = i2c_readNak();
}

void i2c_BMP085_Calculate() {
  int32_t  x1, x2, x3, b3, b5, b6, p, tmp;
  uint32_t b4, b7;
  // Temperature calculations
  x1 = ((int32_t)bmp085_ctx.ut.val - bmp085_ctx.ac6) * bmp085_ctx.ac5 >> 15;
  x2 = ((int32_t)bmp085_ctx.mc << 11) / (x1 + bmp085_ctx.md);
  b5 = x1 + x2;
  baroTemperature = (b5 * 10 + 8) >> 4; // in 0.01 degC (same as MS561101BA temperature)
  // Pressure calculations
  b6 = b5 - 4000;
  x1 = (bmp085_ctx.b2 * (b6 * b6 >> 12)) >> 11; 
  x2 = bmp085_ctx.ac2 * b6 >> 11;
  x3 = x1 + x2;
  tmp = bmp085_ctx.ac1;
  tmp = (tmp*4 + x3) << OSS;
  b3 = (tmp+2)/4;
  x1 = bmp085_ctx.ac3 * b6 >> 13;
  x2 = (bmp085_ctx.b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (bmp085_ctx.ac4 * (uint32_t)(x3 + 32768)) >> 15;
  b7 = ((uint32_t) (bmp085_ctx.up.val >> (8-OSS)) - b3) * (50000 >> OSS);
  p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  baroPressure = p + ((x1 + x2 + 3791) >> 4);
}


//return 0: no data available, no computation ;  1: new value available  ; 2: no new value, but computation time
uint8_t Baro_update() {                   // first UT conversion is started in init procedure
  if (currentTime < bmp085_ctx.deadline) return 0; 
  bmp085_ctx.deadline = currentTime+6000; // 1.5ms margin according to the spec (4.5ms T convetion time)
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz, BMP085 is ok with this speed
  if (bmp085_ctx.state == 0) {
    i2c_BMP085_UT_Read(); 
    i2c_BMP085_UP_Start(); 
    bmp085_ctx.state = 1; 
    Baro_Common();
    bmp085_ctx.deadline += 21000;   // 6000+21000=27000 1.5ms margin according to the spec (25.5ms P convetion time with OSS=3)
    return 1;
  } else {
    i2c_BMP085_UP_Read(); 
    i2c_BMP085_UT_Start(); 
    i2c_BMP085_Calculate(); 
    bmp085_ctx.state = 0; 
    return 2;
  }
}


 
void Baro_Common() {
    static int32_t baroHistTab[BARO_TAB_SIZE];
    static uint8_t baroHistIdx;
  
    uint8_t indexplus1 = (baroHistIdx + 1);//下一个索引
    if (indexplus1 == BARO_TAB_SIZE) 
        indexplus1 = 0;
    baroHistTab[baroHistIdx] = baroPressure;
    baroPressureSum += baroHistTab[baroHistIdx];
    baroPressureSum -= baroHistTab[indexplus1];
    baroHistIdx = indexplus1;  
  }
  
  
  
  
//HMC5883驱动
#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_X_SELF_TEST_GAUSS (+1.16)                       
//!< X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (+1.16)   
//!< Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08)                       
//!< Y axis level when bias current is applied.
#define SELF_TEST_LOW_LIMIT  (243.0/390.0)   //!< Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0/390.0)   //!< High limit when gain is 5.
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

#define MAG_ADDRESS 0x1E                // 7bit
#define MAG_DATA_REGISTER 0x03  
 


void Mag_init() {
  int32_t xyz_total[3]={0,0,0};  // 32 bit totals so they won't overflow.
  bool bret=true;                // Error indicator

  delay(50);  //Wait before start
  i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFA, 0x010 + HMC_POS_BIAS); 
  // Reg A DOR=0x010 + MS1,MS0 set to pos bias

  // Note that the very first measurement after a gain change maintains the same gain as the previous setting. 
  // The new gain setting is effective from the second measurement and on.

  i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFB, 2 << 5);  //Set the Gain
  i2c_writeReg(MAG_ADDRESS,HMC58X3_R_MODE, 1);
  delay(100);
  getADC();  //Get one sample, and discard it

  for (uint8_t i=0; i<10; i++) { //Collect 10 samples
    i2c_writeReg(MAG_ADDRESS,HMC58X3_R_MODE, 1);
    delay(100);
    getADC();   
    // Get the raw values in case the scales have already been changed.
                
    // Since the measurements are noisy, they should be averaged rather than taking the max.
    xyz_total[0]+=imu.magADC[0];
    xyz_total[1]+=imu.magADC[1];
    xyz_total[2]+=imu.magADC[2];
                
    // Detect saturation.
    if (-(1<<12) >= min(imu.magADC[0],min(imu.magADC[1],imu.magADC[2]))) {
      bret=false;
      break;  
      // Breaks out of the for loop.  No sense in continuing if we saturated.
    }
  }

  // Apply the negative bias. (Same gain)
  i2c_writeReg(MAG_ADDRESS,HMC58X3_R_CONFA, 0x010 + HMC_NEG_BIAS); 
  // Reg A DOR=0x010 + MS1,MS0 set to negative bias.
  for (uint8_t i=0; i<10; i++) { 
    i2c_writeReg(MAG_ADDRESS,HMC58X3_R_MODE, 1);
    delay(100);
    getADC();  
    // Get the raw values in case the scales have already been changed.
                
    // Since the measurements are noisy, they should be averaged.
    xyz_total[0]-=imu.magADC[0];
    xyz_total[1]-=imu.magADC[1];
    xyz_total[2]-=imu.magADC[2];

    // Detect saturation.
    if (-(1<<12) >= min(imu.magADC[0],min(imu.magADC[1],imu.magADC[2]))) {
      bret=false;
      break;  
      // Breaks out of the for loop.  No sense in continuing if we saturated.
    }
  }

  magGain[0]=fabs(820.0*HMC58X3_X_SELF_TEST_GAUSS*2.0*10.0/xyz_total[0]);
  magGain[1]=fabs(820.0*HMC58X3_Y_SELF_TEST_GAUSS*2.0*10.0/xyz_total[1]);
  magGain[2]=fabs(820.0*HMC58X3_Z_SELF_TEST_GAUSS*2.0*10.0/xyz_total[2]);

  // leave test mode
  i2c_writeReg(MAG_ADDRESS ,HMC58X3_R_CONFA ,0x70 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
  i2c_writeReg(MAG_ADDRESS ,HMC58X3_R_CONFB ,0x20 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
  i2c_writeReg(MAG_ADDRESS ,HMC58X3_R_MODE  ,0x00 ); //Mode register             -- 000000 00    continuous Conversion Mode
  delay(100);
  magInit = 1;

  if (!bret) { //Something went wrong so get a best guess
    magGain[0] = 1.0;
    magGain[1] = 1.0;
    magGain[2] = 1.0;
  }
} //  Mag_init().


void getADC() {
  i2c_getSixRawADC(MAG_ADDRESS,MAG_DATA_REGISTER);

   MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                    ((rawADC[4]<<8) | rawADC[5]) ,
                    ((rawADC[2]<<8) | rawADC[3]) );
}


void Device_Mag_getADC() {
  getADC();
}



// MPU6050驱动
void Gyro_init() {
  TWBR = ((F_CPU / 400000L) - 16) / 2;                  
  // change the I2C clock rate to 400kHz
  i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x80);             
  //PWR_MGMT_1    -- DEVICE_RESET 1
  delay(5);
  i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x03);             
  //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  i2c_writeReg(MPU6050_ADDRESS, 0x1A, MPU6050_DLPF_CFG); 
  //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
  i2c_writeReg(MPU6050_ADDRESS, 0x1B, 0x18);             
  //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
  // enable I2C bypass for AUX I2C
  i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x02);           
  //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=1 ; CLKOUT_EN=0
}


void Gyro_getADC () {
  i2c_getSixRawADC(MPU6050_ADDRESS, 0x43);
  GYRO_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])>>2 , // range: +/- 8192; +/- 2000 deg/sec
                    ((rawADC[2]<<8) | rawADC[3])>>2 ,
                    ((rawADC[4]<<8) | rawADC[5])>>2 );
  GYRO_Common();
}



void ACC_init () {
  i2c_writeReg(MPU6050_ADDRESS, 0x1C, 0x10);             //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
  //note: something seems to be wrong in the spec here. With AFS=2 1G = 4096 but according to my measurement: 1G=2048 (and 2048/8 = 256)
  //confirmed here: http://www.multiwii.com/forum/viewtopic.php?f=8&t=1080&start=10#p7480

  #if defined(MPU6050_I2C_AUX_MASTER)
    //at this stage, the MAG is configured via the original MAG init function in I2C bypass mode
    //now we configure MPU as a I2C Master device to handle the MAG via the I2C AUX port (done here for HMC5883)
    i2c_writeReg(MPU6050_ADDRESS, 0x6A, 0b00100000);       //USER_CTRL     -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=1 (I2C master mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
    i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x00);             //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=0 ; CLKOUT_EN=0
    i2c_writeReg(MPU6050_ADDRESS, 0x24, 0x0D);             //I2C_MST_CTRL  -- MULT_MST_EN=0 ; WAIT_FOR_ES=0 ; SLV_3_FIFO_EN=0 ; I2C_MST_P_NSR=0 ; I2C_MST_CLK=13 (I2C slave speed bus = 400kHz)
    i2c_writeReg(MPU6050_ADDRESS, 0x25, 0x80|MAG_ADDRESS);//I2C_SLV0_ADDR -- I2C_SLV4_RW=1 (read operation) ; I2C_SLV4_ADDR=MAG_ADDRESS
    i2c_writeReg(MPU6050_ADDRESS, 0x26, MAG_DATA_REGISTER);//I2C_SLV0_REG  -- 6 data bytes of MAG are stored in 6 registers. First register address is MAG_DATA_REGISTER
    i2c_writeReg(MPU6050_ADDRESS, 0x27, 0x86);             //I2C_SLV0_CTRL -- I2C_SLV0_EN=1 ; I2C_SLV0_BYTE_SW=0 ; I2C_SLV0_REG_DIS=0 ; I2C_SLV0_GRP=0 ; I2C_SLV0_LEN=3 (3x2 bytes)
  #endif
}




void ACC_getADC () {
  i2c_getSixRawADC(MPU6050_ADDRESS, 0x3B);
  ACC_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])>>3 ,
                   ((rawADC[2]<<8) | rawADC[3])>>3 ,
                   ((rawADC[4]<<8) | rawADC[5])>>3 );
  ACC_Common();
}







void initSensors() {
  delay(200);
  POWERPIN_ON;
  delay(100);
  i2c_init();
  delay(100);
  Gyro_init();
  Baro_init();
  Mag_init();
  ACC_init();
  //Sonar_init();
  f.I2C_INIT_DONE = 1;
}



























