#include <avr/eeprom.h>//这个实际上用的是avr内部EEPROM
#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "EEPROM.h"
#include "MultiWii.h"
#include "Alarms.h"


void LoadDefaults(void);//函数声明，定义在下面




uint8_t calculate_sum(uint8_t *cb , uint8_t siz) {
  uint8_t sum=0x55;  // checksum init
  while(--siz) sum += *cb++;  // calculate checksum (without checksum byte)
  return sum;
}




void readGlobalSet() {
  eeprom_read_block((void*)&global_conf, (void*)0, sizeof(global_conf));
  // 校验和出错情况处理
  if(calculate_sum((uint8_t*)&global_conf, sizeof(global_conf)) != global_conf.checksum) {
    global_conf.currentSet = 0;
    global_conf.accZero[ROLL] = 5000;    // 配置错误信号
  }
}





bool readEEPROM() {
  uint8_t i;
  global_conf.currentSet=0;
  
  eeprom_read_block((void*)&conf, (void*)(global_conf.currentSet * sizeof(conf) + sizeof(global_conf)), sizeof(conf));
  
  // 校验和出错情况处理
  if(calculate_sum((uint8_t*)&conf, sizeof(conf)) != conf.checksum) {
    blinkLED(6,100,3);    

    LoadDefaults();                 // force load defaults 
    return false;                   // defaults loaded, don't reload constants (EEPROM life saving)
  }
  
  // 下面初始化油门 舵量参考表
  // 500/128 = 3.90625    3.9062 * 3.9062 = 15.259   1526*100/128 = 1192
  for(i=0;i<5;i++) {
    lookupPitchRollRC[i] = (1526+conf.rcExpo8*(i*i-15))*i*(int32_t)conf.rcRate8/1192;
  }
  for(i=0;i<11;i++) {
    int16_t tmp = 10*i-conf.thrMid8;
    uint8_t y = 1;
    if (tmp>0) y = 100-conf.thrMid8;
    if (tmp<0) y = conf.thrMid8;
    lookupThrottleRC[i] = 10*conf.thrMid8 + tmp*( 100-conf.thrExpo8+(int32_t)conf.thrExpo8*(tmp*tmp)/(y*y) )/10; // [0;1000]
    lookupThrottleRC[i] = conf.minthrottle + (int32_t)(MAXTHROTTLE-conf.minthrottle)* lookupThrottleRC[i]/1000;  // [0;1000] -> [conf.minthrottle;MAXTHROTTLE]
  }

  return true;    // 配置正常
}




void writeGlobalSet(uint8_t b) {
  global_conf.checksum = calculate_sum((uint8_t*)&global_conf, sizeof(global_conf));
  eeprom_write_block((const void*)&global_conf, (void*)0, sizeof(global_conf));
  if (b == 1) 
	  blinkLED(15,20,1);
}



void writeParams(uint8_t b) {
  global_conf.currentSet=0;
  conf.checksum = calculate_sum((uint8_t*)&conf, sizeof(conf));
  eeprom_write_block((const void*)&conf, (void*)(global_conf.currentSet * sizeof(conf) + sizeof(global_conf)), sizeof(conf));
  
  readEEPROM();
  if (b == 1) 
	  blinkLED(15,20,1);
}


void update_constants(){
  #if defined(GYRO_SMOOTHING)
    {
      uint8_t s[3] = GYRO_SMOOTHING;
      for(uint8_t i=0;i<3;i++) conf.Smoothing[i] = s[i];
    }
  #endif

  conf.minthrottle = MINTHROTTLE;
  conf.mag_declination = (int16_t)(MAG_DECLINATION * 10);

  writeParams(0);
  }



void LoadDefaults() {
  uint8_t i;

  conf.pid[ROLL].P8     = 28;  conf.pid[ROLL].I8    = 10; conf.pid[ROLL].D8     = 7;
  conf.pid[PITCH].P8    = 28; conf.pid[PITCH].I8    = 10; conf.pid[PITCH].D8    = 7;
  conf.pid[PIDLEVEL].P8 = 30; conf.pid[PIDLEVEL].I8 = 32; conf.pid[PIDLEVEL].D8 = 0;

  conf.pid[YAW].P8      = 68;  conf.pid[YAW].I8     = 45;  conf.pid[YAW].D8     = 0;
  conf.pid[PIDALT].P8   = 64; conf.pid[PIDALT].I8   = 25; conf.pid[PIDALT].D8   = 24;

  conf.pid[PIDPOS].P8  = POSHOLD_P * 100;     conf.pid[PIDPOS].I8    = POSHOLD_I * 100;       conf.pid[PIDPOS].D8    = 0;
  conf.pid[PIDPOSR].P8 = POSHOLD_RATE_P * 10; conf.pid[PIDPOSR].I8   = POSHOLD_RATE_I * 100;  conf.pid[PIDPOSR].D8   = POSHOLD_RATE_D * 1000;
  conf.pid[PIDNAVR].P8 = NAV_P * 10;          conf.pid[PIDNAVR].I8   = NAV_I * 100;           conf.pid[PIDNAVR].D8   = NAV_D * 1000;
  
  conf.pid[PIDMAG].P8   = 40;

  conf.pid[PIDVEL].P8 = 0;      conf.pid[PIDVEL].I8 = 0;    conf.pid[PIDVEL].D8 = 0;

  //舵量曲线控制
  conf.rcRate8 = 90; conf.rcExpo8 = 65;
  conf.rollPitchRate = 0;
  conf.yawRate = 0;
  conf.dynThrPID = 0;
  conf.thrMid8 = 50; conf.thrExpo8 = 0;
	
  for(i=0;i<CHECKBOXITEMS;i++) 
  {
	conf.activate[i] = 0;
  }
    conf.angleTrim[0] = 0; conf.angleTrim[1] = 0;
    conf.powerTrigger1 = 0;

  update_constants(); // this will also write to eeprom
}









