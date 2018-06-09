//接导航板GPS飞控驱动
#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "GPS.h"
#include "Serial.h"
#include "Sensors.h"
#include "MultiWii.h"




// 旋翼到下一个路径点的角度
// 加上交轨偏差 角度*100数值
static int32_t nav_bearing;
// saves the bearing at takeof (1deg = 1) used to rotate to takeoff direction when arrives at home
static int16_t nav_takeoff_bearing;
static void GPS_reset_home_position(void);




// I2C GPS 辅助函数
// 发送一条指令给I2C GPS模块, 先参数指令，再参数路径点数字
void GPS_I2C_command(uint8_t command, uint8_t wp) {
  uint8_t _cmd;
      
  _cmd = (wp << 4) + command;
    
  i2c_rep_start(I2C_GPS_ADDRESS<<1);
  i2c_write(I2C_GPS_COMMAND);
  i2c_write(_cmd);
}



void GPS_NewData(void) {
  uint8_t axis;
  static uint8_t GPS_pids_initialized;
  static uint8_t _i2c_gps_status;
    
  //i2c_stop指令在最后发送的情况下，使用i2c_writereg()函数不起作用
  //Still investigating, however with separated i2c_repstart and i2c_write commands works... and did not caused i2c errors on a long term test.

  uint8_t *varptr;
  
  i2c_rep_start(I2C_GPS_ADDRESS<<1);
  i2c_write(I2C_GPS_STATUS_00);         
  i2c_rep_start((I2C_GPS_ADDRESS<<1)|1);
  
  varptr = (uint8_t *)&_i2c_gps_status;          // 单步驱动读取
  *varptr++ = i2c_readAck();
  *varptr   = i2c_readNak();
  
  GPS_numSat = (_i2c_gps_status & 0xf0) >> 4;
  // GPS当前卫星个数
  
  if (_i2c_gps_status & I2C_GPS_STATUS_3DFIX) {                                     //查看3d定位状态如何(卫星个数：numsats>5)
      f.GPS_FIX = 1;

      if (!f.GPS_FIX_HOME && f.ARMED) {        
      //如果没有设置home的位置，则将home的位置设置为WP#0然后激活它
         GPS_reset_home_position();
      }
      if (_i2c_gps_status & I2C_GPS_STATUS_NEW_DATA) {                                //Check about new data
        if (GPS_update) { GPS_update = 0;} else { GPS_update = 1;}                    //Fancy flash on GUI :D
        if (!GPS_pids_initialized) {
          GPS_set_pids();
          GPS_pids_initialized = 1;
        } 

        i2c_rep_start(I2C_GPS_ADDRESS<<1);
        i2c_write(I2C_GPS_NAV_BEARING);                                               
        //开始读取 2x2 字节距离和朝向数据
        i2c_rep_start((I2C_GPS_ADDRESS<<1)|1);

        varptr = (uint8_t *)&nav_bearing;
        *varptr++ = i2c_readAck();
        *varptr   = i2c_readAck();

        varptr = (uint8_t *)&GPS_directionToHome;
        *varptr++ = i2c_readAck();
        *varptr   = i2c_readAck();
        GPS_directionToHome = GPS_directionToHome / 100;  
        // 读到的寄存器数据1deg =1000数值，缩小处理
        GPS_directionToHome += 180; 
        // 定位 (查看 http://www.multiwii.com/forum/viewtopic.php?f=8&t=2892)
        if (GPS_directionToHome>180) GPS_directionToHome -= 360;

        varptr = (uint8_t *)&GPS_distanceToHome;
        *varptr++ = i2c_readAck();
        *varptr   = i2c_readNak();
        GPS_distanceToHome = GPS_distanceToHome / 100;      
        //寄存器当中单位：CM, 转化成单位：m，这样最大655米（16bit：65535）

        i2c_rep_start(I2C_GPS_ADDRESS<<1);
        i2c_write(I2C_GPS_LOCATION);                
        //开始读取2x2字节坐标
        i2c_rep_start((I2C_GPS_ADDRESS<<1)|1);

        varptr = (uint8_t *)&GPS_coord[LAT];        
        // 纬度显示用
        *varptr++ = i2c_readAck();
        *varptr++ = i2c_readAck();
        *varptr++ = i2c_readAck();
        *varptr   = i2c_readAck();

        varptr = (uint8_t *)&GPS_coord[LON];        
        // 经度显示用
        *varptr++ = i2c_readAck();
        *varptr++ = i2c_readAck();
        *varptr++ = i2c_readAck();
        *varptr   = i2c_readAck();

        varptr = (uint8_t *)&nav[LAT];
        *varptr++ = i2c_readAck();
        *varptr++ = i2c_readAck();

        varptr = (uint8_t *)&nav[LON];
        *varptr++ = i2c_readAck();
        *varptr++ = i2c_readNak();

        i2c_rep_start(I2C_GPS_ADDRESS<<1);
        i2c_write(I2C_GPS_GROUND_SPEED);          
        i2c_rep_start((I2C_GPS_ADDRESS<<1)|1);

        varptr = (uint8_t *)&GPS_speed;          
        *varptr++ = i2c_readAck();
        *varptr   = i2c_readAck();

        varptr = (uint8_t *)&GPS_altitude;       
        *varptr++ = i2c_readAck();
        *varptr   = i2c_readAck();	

        // GPS地面航线
        varptr = (uint8_t *)&GPS_ground_course;
        *varptr++ = i2c_readAck();
        *varptr   = i2c_readNak();

        if (!f.GPS_FIX_HOME) {     
        // 如果没有设置家的位置，无显示
          GPS_distanceToHome = 0;
          GPS_directionToHome = 0;
        }

        // 导航时调整朝向
        if (f.GPS_HOME_MODE) {  
          if ( !(_i2c_gps_status & I2C_GPS_STATUS_WP_REACHED) ) {
            //Tail control
            if (NAV_CONTROLS_HEADING) {
              if (NAV_TAIL_FIRST) {
                magHold = nav_bearing/100-180;
                if (magHold > 180)  magHold -= 360;
                if (magHold < -180) magHold += 360;
              } else {
                magHold = nav_bearing/100;
              }
            }
          } else {        
          //到家了
            if (NAV_SET_TAKEOFF_HEADING) { magHold = nav_takeoff_bearing; }
          }
        }
      }
    } else {
		//We don't have a fix zero out distance and bearing (for safety reasons)
      GPS_distanceToHome = 0;
      GPS_directionToHome = 0;
      GPS_numSat = 0;
      f.GPS_FIX = 0;
    }
}





void GPS_reset_home_position(void) {
  if (f.GPS_FIX && GPS_numSat >= 5) {
    //设置当前位置作为家的位置，WP0
    GPS_I2C_command(I2C_GPS_COMMAND_SET_WP,0); 
	
    nav_takeoff_bearing = att.heading;             
    //保存起飞时朝向
    //设置地面高度
    f.GPS_FIX_HOME = 1;
  }
}




void GPS_reset_nav(void) {
  uint8_t i;
  for(i=0;i<2;i++) {
    nav_rated[i] = 0;
    nav[i] = 0;
	GPS_I2C_command(I2C_GPS_COMMAND_STOP_NAV,0);
  }
}
	
	
	
	
	
	
	


void GPS_set_pids(void) {
       i2c_rep_start(I2C_GPS_ADDRESS<<1);
       i2c_write(I2C_GPS_HOLD_P);
       i2c_write(conf.pid[PIDPOS].P8);
       i2c_write(conf.pid[PIDPOS].I8);
    
    i2c_rep_start(I2C_GPS_ADDRESS<<1);
      i2c_write(I2C_GPS_HOLD_RATE_P);
       i2c_write(conf.pid[PIDPOSR].P8);
       i2c_write(conf.pid[PIDPOSR].I8);
       i2c_write(conf.pid[PIDPOSR].D8);
    
    i2c_rep_start(I2C_GPS_ADDRESS<<1);
      i2c_write(I2C_GPS_NAV_P);
       i2c_write(conf.pid[PIDNAVR].P8);
       i2c_write(conf.pid[PIDNAVR].I8);
       i2c_write(conf.pid[PIDNAVR].D8);
    
    GPS_I2C_command(I2C_GPS_COMMAND_UPDATE_PIDS,0);
    
    uint8_t nav_flags = 0;
	i2c_rep_start(I2C_GPS_ADDRESS<<1);
    i2c_write(I2C_GPS_NAV_FLAGS);
    i2c_write(nav_flags);
      
    i2c_rep_start(I2C_GPS_ADDRESS<<1);
    i2c_write(I2C_GPS_WP_RADIUS);
    i2c_write(GPS_WP_RADIUS & 0x00FF); // lower eight bit   
    i2c_write(GPS_WP_RADIUS >> 8); // upper eight bit
	
}







int32_t wrap_18000(int32_t ang) {
  if (ang > 18000)  ang -= 36000;
  if (ang < -18000) ang += 36000;
  return ang;
}


























