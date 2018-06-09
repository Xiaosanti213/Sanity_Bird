#include <avr/io.h>

#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Alarms.h"
#include "EEPROM.h"
#include "IMU.h"
#include "Output.h"
#include "RX.h"
#include "Sensors.h"
#include "Serial.h"
#include "Protocol.h"
#include "GPS.h"

#include <avr/pgmspace.h>


const char pidnames[] PROGMEM =
  "ROLL;"
  "PITCH;"
  "YAW;"
  "ALT;"
  "Pos;"
  "PosR;"
  "NavR;"
  "LEVEL;"
  "MAG;"
  "VEL;"
;

const char boxnames[] PROGMEM = // names for dynamic generation of config GUI
  "ARM;"
  "ANGLE;"
  "HORIZON;"
  "BARO;"
  "MAG;"
  "HEADFREE;"
  "HEADADJ;"  

;

const uint8_t boxids[] PROGMEM = {// permanent IDs associated to boxes. This way, you can rely on an ID number to identify a BOX function.
  0, //"ARM;"
  1, //"ANGLE;"
  2, //"HORIZON;"
  3, //"BARO;"
  5, //"MAG;"
  6, //"HEADFREE;"
  7, //"HEADADJ;"  
};



int16_t  magHold,headFreeModeHold; // [-180;+180]
uint8_t  vbatMin = 0.1;//VBATNOMINAL;  // lowest battery voltage in 0.1V steps
uint8_t  rcOptions[CHECKBOXITEMS];


int16_t rcSerial[8];         // interval [1000;2000] - 串口发送而来的舵量数据
uint8_t rcSerialCount = 0;   // a counter to select legacy RX when there is no more MSP rc serial data


  int16_t  GPS_angle[2] = { 0, 0};                      // the angles that must be applied for GPS correction
  int32_t  GPS_coord[2];
  int32_t  GPS_home[2];
  int32_t  GPS_hold[2];
  uint8_t  GPS_numSat;
  uint16_t GPS_distanceToHome;                          // distance to home  - unit: meter
  int16_t  GPS_directionToHome;                         // direction to home - unit: degree
  uint16_t GPS_altitude;                                // GPS altitude      - unit: meter
  uint16_t GPS_speed;                                   // GPS speed         - unit: cm/s
  uint8_t  GPS_update = 0;                              // a binary toogle to distinct a GPS position update
  uint16_t GPS_ground_course = 0;                       //                   - unit: degree*10
  uint8_t  GPS_Present = 0;                             // Checksum from Gps serial
  uint8_t  GPS_Enable  = 0;

  // The desired bank towards North (Positive) or South (Negative) : latitude
  // The desired bank towards East (Positive) or West (Negative)   : longitude
  int16_t  nav[2];
  int16_t  nav_rated[2];    //Adding a rate controller to the navigation to make it smoother

  uint8_t nav_mode = NAV_MODE_NONE; // Navigation mode


  int16_t servo[8] = {1500,1500,1500,1500,1500,1500,1500,1000};

  uint16_t intPowerTrigger1;

  //上面是串口发送需要用到的数据，虽然计算时候没用
  /**************************************************************************************************************/


  
uint32_t currentTime = 0;
uint16_t previousTime = 0;
uint16_t cycleTime = 0;  


uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
uint16_t calibratingB = 0;  // baro calibration = get new ground pressure value
uint16_t calibratingG = 0;



int32_t  AltHold; // in cm
int16_t  sonarAlt;
int16_t  BaroPID = 0;
int16_t  errorAltitudeI = 0;



int16_t gyroZero[3] = {0,0,0};      //陀螺仪校准零偏
imu_t imu;							
analog_t analog;
alt_t alt;
att_t att;
int16_t  debug[4];					//调试输出用参数
flags_struct_t f;					//标志位结构体

int16_t  i2c_errors_count = 0;
int16_t  annex650_overrun_count = 0;




//手势判断中使用
#define ROL_LO  (1<<(2*ROLL))		//ROLL=0 在低位为01 中位11 高位10
#define ROL_CE  (3<<(2*ROLL))		
#define ROL_HI  (2<<(2*ROLL))
#define PIT_LO  (1<<(2*PITCH))
#define PIT_CE  (3<<(2*PITCH))
#define PIT_HI  (2<<(2*PITCH))
#define YAW_LO  (1<<(2*YAW))
#define YAW_CE  (3<<(2*YAW))
#define YAW_HI  (2<<(2*YAW))
#define THR_LO  (1<<(2*THROTTLE))
#define THR_CE  (3<<(2*THROTTLE))
#define THR_HI  (2<<(2*THROTTLE))



int16_t rcData[RC_CHANS]; 
int16_t rcCommand[4];               // 对油门范围[1000;2000]，对另外三轴舵量范围[-500;+500]
int16_t lookupPitchRollRC[5];		// 滚转俯仰舵量曲线控制点
int16_t lookupThrottleRC[11];

int16_t axisPID[3];
int16_t motor[8];

static uint8_t dynP8[2], dynD8[2];
global_conf_t global_conf;
conf_t conf;

uint8_t alarmArray[16]; 

int32_t baroPressure;
int32_t baroTemperature;
int32_t baroPressureSum;



void annexCode() { //控制点映射舵量曲线
  static uint32_t calibratedAccTime;
  uint16_t tmp,tmp2;
  uint8_t axis,prop1,prop2;

  // PITCH & ROLL 依据当前油门进行动态PID整定
  prop2 = 128; 
  if (rcData[THROTTLE]>1500) { 
    if (rcData[THROTTLE]<2000) {
      prop2 -=  ((uint16_t)conf.dynThrPID*(rcData[THROTTLE]-1500)>>9); 
    } else {
      prop2 -=  conf.dynThrPID;
    }
  }

  for(axis=0;axis<3;axis++) {
    tmp = min(abs(rcData[axis]-MIDRC),500);
    if(axis!=2) { //ROLL & PITCH
      tmp2 = tmp>>7; // 500/128 = 3.9  => range [0;3]
      rcCommand[axis] = lookupPitchRollRC[tmp2] + ((tmp-(tmp2<<7)) * (lookupPitchRollRC[tmp2+1]-lookupPitchRollRC[tmp2])>>7);
      
	  prop1 = 128-((uint16_t)conf.rollPitchRate*tmp>>9); // prop1 was 100, is 128 now -- and /512 instead of /500
      prop1 = (uint16_t)prop1*prop2>>7; // prop1: max is 128   prop2: max is 128   result prop1: max is 128
      dynP8[axis] = (uint16_t)conf.pid[axis].P8*prop1>>7; // was /100, is /128 now
      dynD8[axis] = (uint16_t)conf.pid[axis].D8*prop1>>7; // was /100, is /128 now
    } else {      // YAW
      rcCommand[axis] = tmp;
    }
    if (rcData[axis]<MIDRC) 
        rcCommand[axis] = -rcCommand[axis];
  }
  
  tmp = constrain(rcData[THROTTLE],MINCHECK,2000);
  tmp = (uint32_t)(tmp-MINCHECK)*2559/(2000-MINCHECK); // [MINCHECK;2000] -> [0;2559]
  tmp2 = tmp/256; // range [0;9]
  rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp-tmp2*256) * (lookupThrottleRC[tmp2+1]-lookupThrottleRC[tmp2]) / 256; 
  // [0;2559] -> expo -> [conf.minthrottle;MAXTHROTTLE]


  serialCom();




  //校准状态等
  if ( (calibratingA>0) || (calibratingG>0) ) { // 进入下次循环，仍需要校准
    LEDPIN_TOGGLE;
  } else {
    if (f.ACC_CALIBRATED) {LEDPIN_OFF;}//加计校准后LED灯灭
    if (f.ARMED) {LEDPIN_ON;}//解锁状态灯亮
  }

  if ( currentTime > calibratedAccTime ) {
    if (! f.SMALL_ANGLES_25) {
      // the multi uses ACC and is not calibrated or is too much inclinated 
      f.ACC_CALIBRATED = 0;
      LEDPIN_TOGGLE;
      calibratedAccTime = currentTime + 100000;//100ms之后
    } else {
      f.ACC_CALIBRATED = 1;//大角度倾斜后需要重新校准
    }
  }




  
}



void setup() {

  SerialOpen(0,SERIAL0_COM_SPEED);
  //文件def.h
  LEDPIN_PINMODE;
  POWERPIN_PINMODE;
  BUZZERPIN_PINMODE;
  STABLEPIN_PINMODE;
  POWERPIN_OFF;
  
  initOutput();
  
  readGlobalSet();
  readEEPROM();                               // check current setting integrity 
  blinkLED(2,40,1);       
  
  configureReceiver();						  // 配置接收机引脚
 
  initSensors();							  // 初始化IMU
  
  GPS_set_pids();							  // 设置GPS的PID参数
  previousTime = micros();

  //貌似是循环过程中动态校准，这个赋值好像需要在初始化传感器前面
  calibratingG = 512;
  calibratingB = 200;  // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles

  f.SMALL_ANGLES_25=1; 
    // important for gyro only conf
  
}



void go_arm() {
  if(calibratingG == 0 
    && f.ACC_CALIBRATED 
  ){
    if(!f.ARMED) { // arm now!
      f.ARMED = 1;
      //magHold = att.heading;

    }
    
  } 
  else if(!f.ARMED) { // 状态不正常 无法解锁
    blinkLED(2,255,1);
    alarmArray[8] = 1;
  }
}



void go_disarm() {
  if (f.ARMED) {
    f.ARMED = 0;
  }
}





void loop () {
	
  static uint8_t rcDelayCommand; 
  // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
  static uint8_t rcSticks;       
  // this hold sticks position for command combos
  uint8_t axis,i;
  int16_t error,errorAngle;
  int16_t delta;
  int16_t PTerm = 0,ITerm = 0,DTerm, PTermACC, ITermACC;
  static int16_t lastGyro[2] = {0,0};
  static int16_t errorAngleI[2] = {0,0};
  
  //控制器需要的变量
  static int16_t delta1[3],delta2[3];
  static int32_t errorGyroI[3] = {0,0,0};
  static int16_t lastError[3] = {0,0,0};
  int16_t deltaSum;
  int16_t AngleRateTmp, RateError;

  static uint32_t rcTime  = 0; //确保接收遥控器信号时间阈值
  static int16_t initialThrottleHold;
  static uint32_t timestamp_fixated = 0;
  int16_t rc;
  int32_t prop = 0;	
	
	
	
	
if (currentTime > rcTime ) { // 50Hz
    rcTime = currentTime + 20000;
    computeRC();

	// 检查摇杆位置
    uint8_t stTmp = 0;
    for(i=0;i<4;i++) {                          // 检查四路通道位置
      stTmp >>= 2;                              // 这个右移是 使用8bit迭代标志 7b6b低高 5b4b 3b2b 1b0b四组
      if(rcData[i] > MINCHECK) stTmp |= 0x80;   // 如果不是最低位，将标志位置1
      if(rcData[i] < MAXCHECK) stTmp |= 0x40;   // 如果不是最高位，将标志位置1
    }
	
    if(stTmp == rcSticks) {                 	// 和之前一次的比较，相等则计数
      if(rcDelayCommand<250) 
        rcDelayCommand++;
    } else 
        rcDelayCommand = 0;                		// 中间一旦出现一次非极限位置，清0
		
		rcSticks = stTmp;

	if (rcData[THROTTLE] <= MINCHECK) {         // 油门量小于最低值 积分量都置零
        errorGyroI[ROLL] = 0; errorGyroI[PITCH] = 0;
        errorGyroI[YAW] = 0;
        errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
	}
	
	if(rcDelayCommand == 20) {
      if(f.ARMED) {        						// 如果已经解锁。检查是否上锁                             
          if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) 
			  go_disarm();    					// 通过检测标志位,确定前一次的摇杆位置 
      }
	  else if (rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE) 
		  go_arm();      						// 上锁状态检查是否解锁
	  } else {}
	  
	  
	  
	  static uint8_t GPSNavReset = 1;
	  if (f.GPS_FIX && GPS_numSat >= 5 ) {		// 当前GPS卫星个数足够定点
        if (abs(rcCommand[ROLL])< AP_MODE && abs(rcCommand[PITCH]) < AP_MODE) {
          if (!f.GPS_HOLD_MODE) {
            f.GPS_HOLD_MODE = 1;				// 提取出来只有定点模式
            GPSNavReset = 0;					// 禁止重置导航参数
			GPS_I2C_command(I2C_GPS_COMMAND_POSHOLD,0);
          }
        } 
	  }else{
        f.GPS_HOLD_MODE = 0;
	  }
	  
	  
	  
	  
}
else{
	static uint8_t taskOrder=0; 				// 下面的功能不要在同一个循环当中调用，避免阻塞
    if(taskOrder>3) taskOrder-=4;
    switch (taskOrder) {
      case 0:
        taskOrder++;
        if (Mag_getADC()) break; 				// 最大 350 µs (HMC5883) 获取数据后跳出循环
      case 1:
        taskOrder++;
        if (Baro_update() != 0 ) break;
      case 2:
        taskOrder++;
        if (getEstimatedAltitude() !=0) break;
	  case 3:
        taskOrder++;
		if(GPS_Enable) GPS_NewData();  break;
}

}
	

computeIMU();
// 姿态解算后立刻测循环时间，当前时间，判断循环时使用
currentTime = micros();
// us为单位
cycleTime = currentTime - previousTime;
// 这个应该就是循环过程执行时间28000,
previousTime = currentTime;


#define GYRO_I_MAX 256
#define ACC_I_MAX 256
prop = min(max(abs(rcCommand[PITCH]),abs(rcCommand[ROLL])),500); 
// 俯仰偏航舵量映射指令范围 [0;500]



// 如果当前卫星状态良好，进行HOLD定点并进行下面的运算
if (f.GPS_HOLD_MODE) {
  // 和北向组成直角三角形通过三角函数求两个直角边的长度
  float sin_yaw_y = sin(att.heading*0.0174532925f);
  float cos_yaw_x = cos(att.heading*0.0174532925f);
	  
  // 根据config.h文件当中的定义NAV_SLEW_RATE为30
  nav_rated[LON]   += constrain(wrap_18000(nav[LON]-nav_rated[LON]),-NAV_SLEW_RATE,NAV_SLEW_RATE);
  nav_rated[LAT]   += constrain(wrap_18000(nav[LAT]-nav_rated[LAT]),-NAV_SLEW_RATE,NAV_SLEW_RATE);
  GPS_angle[ROLL]   = (nav_rated[LON]*cos_yaw_x - nav_rated[LAT]*sin_yaw_y) /10;
  GPS_angle[PITCH]  = (nav_rated[LON]*sin_yaw_y + nav_rated[LAT]*cos_yaw_x) /10;
}
	  



//控制  
 for(axis=0;axis<3;axis++) {
    // 外环
    if ( axis<2 ) { 
      // 计算外环偏差，限制舵量指令（即为角度指令）到最大50度倾斜
      // 三部分组成：指令-当前解算姿态角+角度微调量
      // 暂时把GPS_angle[axis]去掉
      errorAngle = constrain((rcCommand[axis]<<1)+ GPS_angle[axis],-500,+500) - att.angle[axis] + conf.angleTrim[axis]; //16位在此处足够存储 
      // 十个舵量代表1度
	  AngleRateTmp = ((int32_t) errorAngle * conf.pid[PIDLEVEL].P8)>>4;//LEVEL外环系数 当前conf.pid[PIDLEVEL].P8 = 3 再除以16 相当于除以5.3  假设errorAngle为几十的量级（角度相差几度）得到十几
    }
	else {
    // 偏航控制：将磁罗盘校正用于控制量当中
    // 舵量直接就是角速率指令
      AngleRateTmp = (((int32_t) (conf.yawRate + 27) * rcCommand[2]) >> 5);//GUI中配置conf.yawRate=0 rcCommand[2]为-500~500 除以32应该是瞬时范围-422~422
    }
	
	
	// 外环角度环作为内环参考，和当前解算速率值作差
	RateError = AngleRateTmp  - imu.gyroData[axis]; //这个imu.gyroData[axis]应该是deg/s
	
	//下面计算内环PID
	
	//内环比例P
	PTerm = ((int32_t) RateError * conf.pid[axis].P8)>>7;  //俯仰滚转轴为2.8 偏航轴为6.8 除以128 得到分别乘0.021875 和 0.053125
	
	//内环积分I 
	errorGyroI[axis]  += (((int32_t) RateError * cycleTime)>>11) * conf.pid[axis].I8;  // 俯仰滚转轴为0.010 偏航轴为0.045 cycleTime微秒为单位28 000数量级 系数相乘得到数量级：0.15
	errorGyroI[axis]  = constrain(errorGyroI[axis], (int32_t) -GYRO_I_MAX<<13, (int32_t) +GYRO_I_MAX<<13);  //GYRO_I_MAX在loop函数中 进入控制器前声明为256 再×8192
	ITerm = errorGyroI[axis]>>13; 
	  
	//内环微分D
	delta = RateError - lastError[axis];  
	lastError[axis] = RateError;  
	delta = ((int32_t) delta * ((uint16_t)0xFFFF / (cycleTime>>4)))>>6;//这个可以用C语言测试输出一下
	//取均值降噪
	deltaSum       = delta1[axis]+delta2[axis]+delta;
	delta2[axis]   = delta1[axis];
	delta1[axis]   = delta;
	DTerm = (deltaSum*conf.pid[axis].D8)>>8;//经过输出测试 始终维持才几的数量级 特别快了会达到30~40
	
	
	//求和计算PID输出
    axisPID[axis] =  PTerm + ITerm + DTerm;


  
  if(axis == 0)
  {
    debug[0] = PTerm;
    debug[1] = ITerm;
  }
  else if(axis == 2)
  {
    debug[2] = PTerm;
    debug[3] = ITerm;
  }



	
	
	mixTable();
	writeMotors();
	
}	


}	
	 






