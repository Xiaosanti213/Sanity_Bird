#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "IMU.h"
#include "Sensors.h"


void getEstimatedAttitude();

void computeIMU () {
  uint8_t axis;
  static int16_t gyroADCprevious[3] = {0,0,0};
  int16_t gyroADCp[3];
  int16_t gyroADCinter[3];
  static uint32_t timeInterleave = 0;
  
  
  ACC_getADC();
  getEstimatedAttitude();//执行姿态解算
  Gyro_getADC();
    
  for (axis = 0; axis < 3; axis++)
    gyroADCp[axis] =  imu.gyroADC[axis];
    // 存储当前数据
  timeInterleave=micros();
  annexCode();
  
  //这块时间控制十分精确
  uint8_t t=0;
  while((uint16_t)(micros()-timeInterleave)<650) 
    t=1; 
  //经验上看, 两次连续读取的交叉存储间隔，存储超时
  if (!t) annex650_overrun_count++;
  
  Gyro_getADC();
  
  for (axis = 0; axis < 3; axis++) {
      gyroADCinter[axis] =  imu.gyroADC[axis]+gyroADCp[axis];
      // 经验上看, 在当前值和前一次值上做了一个加权
      imu.gyroData[axis] = (gyroADCinter[axis]+gyroADCprevious[axis])/3;
      gyroADCprevious[axis] = gyroADCinter[axis]>>1;
      if (!ACC) imu.accADC[axis]=0;
  }
  
  //陀螺仪平滑处理
  /*
  static int16_t gyroSmooth[3] = {0,0,0};
    for (axis = 0; axis < 3; axis++) {
      imu.gyroData[axis] = (int16_t) ( ( (int32_t)((int32_t)gyroSmooth[axis] * (conf.Smoothing[axis]-1) )+imu.gyroData[axis]+1 ) / conf.Smoothing[axis]);
      gyroSmooth[axis] = imu.gyroData[axis];
    }
	*/
}
	
	
	
#ifndef ACC_LPF_FACTOR
  #define ACC_LPF_FACTOR 4 // that means a LPF of 16
#endif

/* Set the Gyro Weight for Gyro/Acc complementary filter
   Increasing this value would reduce and delay Acc influence on the output of the filter*/
#ifndef GYR_CMPF_FACTOR
  #define GYR_CMPF_FACTOR 600
#endif

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter
   Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
#define GYR_CMPFM_FACTOR 250
	
	


#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))

typedef struct fp_vector {		
  float X,Y,Z;		
} t_fp_vector_def;

typedef union {		
  float A[3];		
  t_fp_vector_def V;		
} t_fp_vector;

typedef struct int32_t_vector {
  int32_t X,Y,Z;
} t_int32_t_vector_def;

typedef union {
  int32_t A[3];
  t_int32_t_vector_def V;
} t_int32_t_vector;

int16_t _atan2(int32_t y, int32_t x){
  float z = (float)y / x;
  int16_t a;
  if ( abs(y) < abs(x) ){
     a = 573 * z / (1.0f + 0.28f * z * z);
   if (x<0) {
     if (y<0) a -= 1800;
     else a += 1800;
   }
  } else {
   a = 900 - 573 * z / (z * z + 0.28f);
   if (y<0) a -= 1800;
  }
  return a;
}

float InvSqrt (float x){ 
  union{  
    int32_t i;  
    float   f; 
  } conv; 
  conv.f = x; 
  conv.i = 0x5f3759df - (conv.i >> 1); 
  return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}

// 根据陀螺仪数据，使用小角度估计旋转估计矢量
   void rotateV(struct fp_vector *v,float* delta) {
  fp_vector v_tmp = *v;
  v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
  v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
  v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X;
}


static int32_t accLPF32[3]    = {0, 0, 1};
static float invG; // 1/|G|

static t_fp_vector EstG;
static t_int32_t_vector EstG32;

static t_int32_t_vector EstM32;
static t_fp_vector EstM;




void getEstimatedAttitude(){
  uint8_t axis;
  int32_t accMag = 0;
  float scale, deltaGyroAngle[3];
  uint8_t validAcc;
  static uint16_t previousT;
  uint16_t currentT = micros();

  scale = (currentT - previousT) * GYRO_SCALE; // GYRO_SCALE 单位: rad/us
  previousT = currentT;

  // 【1】初始化 imu.gyroADC imu.accADC都是传感器原始数据，计算旋转角
  for (axis = 0; axis < 3; axis++) {
    deltaGyroAngle[axis] = imu.gyroADC[axis]  * scale; // 单位：rad
    // 这个是陀螺仪数据乘放大倍率（单位1表示的角度）得到三轴角度变化

    accLPF32[axis]    -= accLPF32[axis]>>ACC_LPF_FACTOR;
    accLPF32[axis]    += imu.accADC[axis];
    imu.accSmooth[axis]    = accLPF32[axis]>>ACC_LPF_FACTOR;
    // 一个低通滤波（像是使用多项式平滑处理）

    accMag += (int32_t)imu.accSmooth[axis]*imu.accSmooth[axis] ;
    // 平滑之后计算加计模长
  }

  // 【2】由前一次的姿态矢量，经过小角度旋转近似，得到当前的姿态矢量
  rotateV(&EstG.V,deltaGyroAngle);
  rotateV(&EstM.V,deltaGyroAngle);

  accMag = accMag*100/((int32_t)ACC_1G*ACC_1G);//这个×100的步骤也是为了放大误差
  validAcc = 72 < (uint16_t)accMag && (uint16_t)accMag < 133;


//现在这个.A成员进行了加权处理 
for (axis = 0; axis < 3; axis++) {
    if ( validAcc )//加计数据有效，使用加计修正
      EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + imu.accSmooth[axis]) * INV_GYR_CMPF_FACTOR;
    EstG32.A[axis] = EstG.A[axis]; 
    //int32_t cross calculation is a little bit faster than float	
    //有磁罗盘使用磁罗盘修正
    EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR  + imu.magADC[axis]) * INV_GYR_CMPFM_FACTOR;
    EstM32.A[axis] = EstM.A[axis];
}




// 判断当前倾角是否超过25度
  if ((int16_t)EstG32.A[2] > ACCZ_25deg)
    f.SMALL_ANGLES_25 = 1;
  else
    f.SMALL_ANGLES_25 = 0;



// 【4】估计姿态角 用G成员算
  int32_t sqGX_sqGZ = sq(EstG32.V.X) + sq(EstG32.V.Z);
  invG = InvSqrt(sqGX_sqGZ + sq(EstG32.V.Y));//矢量模长倒数分之一
  att.angle[ROLL]  = _atan2(EstG32.V.X , EstG32.V.Z);
  att.angle[PITCH] = _atan2(EstG32.V.Y , InvSqrt(sqGX_sqGZ)*sqGX_sqGZ);


att.heading = _atan2(
      EstM32.V.Z * EstG32.V.X - EstM32.V.X * EstG32.V.Z,
      (EstM.V.Y * sqGX_sqGZ  - (EstM32.V.X * EstG32.V.X + EstM32.V.Z * EstG32.V.Z) * EstG.V.Y)*invG ); 
    att.heading += conf.mag_declination; // GUI当中设置偏移量
    att.heading /= 10;


    att.heading = 0;

}



#define UPDATE_INTERVAL 25000    // 40hz 更新速率 (20hz LPF on acc)
#define BARO_TAB_SIZE   21

#define ACC_Z_DEADBAND (ACC_1G>>5) // was 40 instead of 32 now


#define applyDeadband(value, deadband)  \
  if(abs(value) < deadband) {           \
    value = 0;                          \
  } else if(value > 0){                 \
    value -= deadband;                  \
  } else if(value < 0){                 \
    value += deadband;                  \
  }
  
  
//高度估计函数 
uint8_t getEstimatedAltitude(){
  int32_t  BaroAlt;
  static float baroGroundTemperatureScale,logBaroGroundPressureSum;
  static float vel = 0.0f;
  static uint16_t previousT;
  uint16_t currentT = micros();
  uint16_t dTime;

  dTime = currentT - previousT;
  if (dTime < UPDATE_INTERVAL) return 0;
  previousT = currentT;

  if(calibratingB > 0) {
    logBaroGroundPressureSum = log(baroPressureSum);
    baroGroundTemperatureScale = (baroTemperature + 27315) *  29.271267f;
    calibratingB--;
  }

  // baroGroundPressureSum is not supposed to be 0 here
  // see: https://code.google.com/p/ardupilot-mega/source/browse/libraries/AP_Baro/AP_Baro.cpp
  BaroAlt = ( logBaroGroundPressureSum - log(baroPressureSum) ) * baroGroundTemperatureScale;

  alt.EstAlt = (alt.EstAlt * 6 + BaroAlt * 2) >> 3; // additional LPF to reduce baro noise (faster by 30 µs)

  #if (defined(VARIOMETER) && (VARIOMETER != 2)) || !defined(SUPPRESS_BARO_ALTHOLD)
    //P
    int16_t error16 = constrain(AltHold - alt.EstAlt, -300, 300);
    applyDeadband(error16, 10); //remove small P parametr to reduce noise near zero position
    BaroPID = constrain((conf.pid[PIDALT].P8 * error16 >>7), -150, +150);

    //I
    errorAltitudeI += conf.pid[PIDALT].I8 * error16 >>6;
    errorAltitudeI = constrain(errorAltitudeI,-30000,30000);
    BaroPID += errorAltitudeI>>9; //I in range +/-60
 
    // projection of ACC vector to global Z, with 1G subtructed
    // Math: accZ = A * G / |G| - 1G
    int16_t accZ = (imu.accSmooth[ROLL] * EstG32.V.X + imu.accSmooth[PITCH] * EstG32.V.Y + imu.accSmooth[YAW] * EstG32.V.Z) * invG;

    static int16_t accZoffset = 0;
    if (!f.ARMED) {
      accZoffset -= accZoffset>>3;
      accZoffset += accZ;
    }  
    accZ -= accZoffset>>3;
    applyDeadband(accZ, ACC_Z_DEADBAND);

    static int32_t lastBaroAlt;
    //int16_t baroVel = (alt.EstAlt - lastBaroAlt) * 1000000.0f / dTime;
    int16_t baroVel = (alt.EstAlt - lastBaroAlt) * (1000000 / UPDATE_INTERVAL);
    lastBaroAlt = alt.EstAlt;

    baroVel = constrain(baroVel, -300, 300); // constrain baro velocity +/- 300cm/s
    applyDeadband(baroVel, 10); // to reduce noise near zero

    // Integrator - velocity, cm/sec
    vel += accZ * ACC_VelScale * dTime;

    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity). 
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    vel = vel * 0.985f + baroVel * 0.015f;

    //D
    alt.vario = vel;
    applyDeadband(alt.vario, 5);
    BaroPID -= constrain(conf.pid[PIDALT].D8 * alt.vario >>4, -150, 150);
  #endif
  return 1;
}
  
  
  
  
  
  
  
  
  















































	
	
	
	
	
	
  
  
