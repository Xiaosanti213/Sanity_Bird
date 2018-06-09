#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "EEPROM.h"
#include "Output.h"
#include "MultiWii.h"
#include "Serial.h"
#include "Protocol.h"
#include "RX.h"

/************************************** MultiWii Serial Protocol *******************************************************/
// Multiwii Serial Protocol 0 
#define MSP_VERSION              0

//to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes
#define MSP_SERVO_CONF           120   //out message         Servo settings

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction
#define MSP_SET_SERVO_CONF       212   //in message          Servo settings
#define MSP_SET_MOTOR            214   //in message          PropBalance function

#define MSP_BIND                 240   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4



static uint8_t CURRENTPORT=0;

#define INBUF_SIZE 64
static uint8_t inBuf[INBUF_SIZE][UART_NUMBER];
static uint8_t checksum[UART_NUMBER];
static uint8_t indRX[UART_NUMBER];
static uint8_t cmdMSP[UART_NUMBER];

void evaluateCommand();


#define BIND_CAPABLE 0; 


const uint32_t capability = 0+BIND_CAPABLE;

uint8_t read8()  {
  return inBuf[indRX[CURRENTPORT]++][CURRENTPORT]&0xff;
}
uint16_t read16() {
  uint16_t t = read8();
  t+= (uint16_t)read8()<<8;
  return t;
}
uint32_t read32() {
  uint32_t t = read16();
  t+= (uint32_t)read16()<<16;
  return t;
}




void serialize8(uint8_t a) {
  SerialSerialize(CURRENTPORT,a);
  checksum[CURRENTPORT] ^= a;
}
void serialize16(int16_t a) {
  serialize8((a   ) & 0xFF);
  serialize8((a>>8) & 0xFF);
}
void serialize32(uint32_t a) {
  serialize8((a    ) & 0xFF);
  serialize8((a>> 8) & 0xFF);
  serialize8((a>>16) & 0xFF);
  serialize8((a>>24) & 0xFF);
}






void headSerialResponse(uint8_t err, uint8_t s) {
  serialize8('$');
  serialize8('M');
  serialize8(err ? '!' : '>');
  checksum[CURRENTPORT] = 0; // start calculating a new checksum
  serialize8(s);
  serialize8(cmdMSP[CURRENTPORT]);
}

void headSerialReply(uint8_t s) {
  headSerialResponse(0, s);
}

void inline headSerialError(uint8_t s) {
  headSerialResponse(1, s);
}

void tailSerialReply() {
  serialize8(checksum[CURRENTPORT]);UartSendData(CURRENTPORT);
}

void serializeNames(PGM_P s) {
  headSerialReply(strlen_P(s));
  for (PGM_P c = s; pgm_read_byte(c); c++) {
    serialize8(pgm_read_byte(c));
  }
}




//主函数接口
void serialCom() {
  uint8_t c,n;  
  static uint8_t offset[UART_NUMBER];
  static uint8_t dataSize[UART_NUMBER];
  static enum _serial_state {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  } c_state[UART_NUMBER];// = IDLE;

  for(n=0;n<UART_NUMBER;n++) {
	  
	  
    #if !defined(PROMINI)
      CURRENTPORT=n;
    #endif
    #define GPS_COND
    #if defined(GPS_SERIAL)
      #if defined(GPS_PROMINI)
        #define GPS_COND       
      #else
        #undef GPS_COND
        #define GPS_COND  && (GPS_SERIAL != CURRENTPORT)
      #endif      
    #endif
    #define SPEK_COND
    #if defined(SPEKTRUM) && (UART_NUMBER > 1)
      #define SPEK_COND && (SPEK_SERIAL_PORT != CURRENTPORT)
    #endif
    #define SBUS_COND 
    #if defined(SBUS) && (UART_NUMBER > 1)
      #define SBUS_COND && (SBUS_SERIAL_PORT != CURRENTPORT)
    #endif
	
	
	
	
    uint8_t cc = SerialAvailable(CURRENTPORT);
    while (cc-- GPS_COND SPEK_COND SBUS_COND) {
      // 循环处理串口接收到的字符组
	  uint8_t bytesTXBuff = SerialUsedTXBuff(CURRENTPORT); // indicates the number of occupied bytes in TX buffer
      if (bytesTXBuff > TX_BUFFER_SIZE - 50 ) return; // ensure there is enough free TX buffer to go further (50 bytes margin)
      c = SerialRead(CURRENTPORT);

  
        // regular data handling to detect and handle MSP and other data
        if (c_state[CURRENTPORT] == IDLE) {//前一次
          c_state[CURRENTPORT] = (c=='$') ? HEADER_START : IDLE;  // 当前
          if (c_state[CURRENTPORT] == IDLE);// evaluateOtherData(c); // evaluate all other incoming serial data
        } else if (c_state[CURRENTPORT] == HEADER_START) {
          c_state[CURRENTPORT] = (c=='M') ? HEADER_M : IDLE;
        } else if (c_state[CURRENTPORT] == HEADER_M) {
          c_state[CURRENTPORT] = (c=='<') ? HEADER_ARROW : IDLE;
        } else if (c_state[CURRENTPORT] == HEADER_ARROW) {
          if (c > INBUF_SIZE) {  // now we are expecting the payload size
            c_state[CURRENTPORT] = IDLE;
            continue;
          }
          dataSize[CURRENTPORT] = c;
          offset[CURRENTPORT] = 0;
          checksum[CURRENTPORT] = 0;
          indRX[CURRENTPORT] = 0;
          checksum[CURRENTPORT] ^= c;
          c_state[CURRENTPORT] = HEADER_SIZE;  // the command is to follow
        
		
		} else if (c_state[CURRENTPORT] == HEADER_SIZE) {
          cmdMSP[CURRENTPORT] = c;
          checksum[CURRENTPORT] ^= c;
          c_state[CURRENTPORT] = HEADER_CMD;
        } else if (c_state[CURRENTPORT] == HEADER_CMD && offset[CURRENTPORT] < dataSize[CURRENTPORT]) {
          checksum[CURRENTPORT] ^= c;
          inBuf[offset[CURRENTPORT]++][CURRENTPORT] = c;
        } else if (c_state[CURRENTPORT] == HEADER_CMD && offset[CURRENTPORT] >= dataSize[CURRENTPORT]) {
          if (checksum[CURRENTPORT] == c) {  // 对比传输和计算校验和是否相同
            evaluateCommand();  // 获得一个有效的数据包，下面进行解析
          }
          c_state[CURRENTPORT] = IDLE;
          cc = 0; // no more than one MSP per port and per cycle
        }

    }
  }
}







void  s_struct(uint8_t *cb,uint8_t siz) {
  headSerialReply(siz);
  while(siz--) serialize8(*cb++);
}

void s_struct_w(uint8_t *cb,uint8_t siz) {
 headSerialReply(0);
  while(siz--) *cb++ = read8();
}





void evaluateCommand() {
  uint32_t tmp=0; 

  switch(cmdMSP[CURRENTPORT]) {
   case MSP_SET_RAW_RC:
     s_struct_w((uint8_t*)&rcSerial,16);
     rcSerialCount = 50; // 1s transition 
     break;

   case MSP_SET_PID:
     s_struct_w((uint8_t*)&conf.pid[0].P8,3*PIDITEMS);
     break;
   case MSP_SET_BOX:
     s_struct_w((uint8_t*)&conf.activate[0],CHECKBOXITEMS*2);
     break;
   case MSP_SET_RC_TUNING:
     s_struct_w((uint8_t*)&conf.rcRate8,7);
     break;
	 

   case MSP_SET_MISC:
     struct {
       uint16_t a,b,c,d,e,f;
       uint32_t g;
       uint16_t h;
       uint8_t  i,j,k,l;
     } set_misc;
     s_struct_w((uint8_t*)&set_misc,22);
 
       conf.mag_declination = set_misc.h;

     break;
   case MSP_MISC:
     struct {
       uint16_t a,b,c,d,e,f;
       uint32_t g;
       uint16_t h;
       uint8_t  i,j,k,l;
     } misc;
     misc.a = intPowerTrigger1;
     misc.b = conf.minthrottle;
     misc.c = MAXTHROTTLE;
     misc.d = MINCOMMAND;
 
     misc.e = 0;

     misc.f = 0; misc.g =0;
     
	 misc.h = conf.mag_declination;
       

     misc.i = 0;misc.j = 0;misc.k = 0;misc.l = 0;

     s_struct((uint8_t*)&misc,22);//发送结构体包
     break;
   
     case MSP_SET_MOTOR:
       s_struct_w((uint8_t*)&motor,16);
     break;

	 
   case MSP_SET_HEAD:
     s_struct_w((uint8_t*)&magHold,2);
     break;
   case MSP_IDENT:
     struct {
       uint8_t v,t,msp_v;
       uint32_t cap;
     } id;
     id.v     = VERSION;
     id.t     = MULTITYPE;
     id.msp_v = MSP_VERSION;
     id.cap   = capability|DYNBAL<<2|FLAP<<3;
     s_struct((uint8_t*)&id,7);
     break;
   case MSP_STATUS:
     struct {
       uint16_t cycleTime,i2c_errors_count,sensor;
       uint32_t flag;
       uint8_t set;
     } st;
     st.cycleTime        = cycleTime;
     st.i2c_errors_count = i2c_errors_count;
     st.sensor           = ACC|BARO<<1|MAG<<2|GPS<<3|SONAR<<4;
     if(f.ANGLE_MODE)   tmp |= 1<<BOXANGLE;
     if(f.HORIZON_MODE) tmp |= 1<<BOXHORIZON;

     if(f.BARO_MODE) tmp |= 1<<BOXBARO;
     
	 if(f.MAG_MODE) tmp |= 1<<BOXMAG;
       
     if(f.ARMED) tmp |= 1<<BOXARM;
     st.flag             = tmp;
     st.set              = global_conf.currentSet;
     s_struct((uint8_t*)&st,11);
     break;
   case MSP_RAW_IMU: 
     s_struct((uint8_t*)&imu,18);//传感器原生数据
     break;
   case MSP_SERVO:
     s_struct((uint8_t*)&servo,16);
     break;
   case MSP_SERVO_CONF:
     s_struct((uint8_t*)&conf.servoConf[0].min,56); // struct servo_conf_ is 7 bytes length: min:2 / max:2 / middle:2 / rate:1    ----     8 servo =>  8x7 = 56
     break;
   case MSP_SET_SERVO_CONF:
     s_struct_w((uint8_t*)&conf.servoConf[0].min,56);
     break;
   case MSP_MOTOR:
     s_struct((uint8_t*)&motor,16);
     break;
   case MSP_RC:
     s_struct((uint8_t*)&rcData,RC_CHANS*2);
     break;

	 
   case MSP_ATTITUDE:
     s_struct((uint8_t*)&att,6);//解算得到的姿态数据
     break;
   case MSP_ALTITUDE:
     s_struct((uint8_t*)&alt,6);
     break;
   case MSP_ANALOG:
     s_struct((uint8_t*)&analog,7);
     break;
   case MSP_RC_TUNING:
     s_struct((uint8_t*)&conf.rcRate8,7);
     break;
   case MSP_PID:
     s_struct((uint8_t*)&conf.pid[0].P8,3*PIDITEMS);
     break;
   case MSP_PIDNAMES:
     serializeNames(pidnames);
     break;
   case MSP_BOX:
     s_struct((uint8_t*)&conf.activate[0],2*CHECKBOXITEMS);
     break;
   case MSP_BOXNAMES:
     serializeNames(boxnames);
     break;
   case MSP_BOXIDS:
     headSerialReply(CHECKBOXITEMS);
     for(uint8_t i=0;i<CHECKBOXITEMS;i++) {
       serialize8(pgm_read_byte(&(boxids[i])));
     }
     break;
   case MSP_MOTOR_PINS:
     s_struct((uint8_t*)&PWM_PIN,8);
     break;
    
   case MSP_WP:
     {
       int32_t lat = 0,lon = 0;
       uint8_t wp_no = read8();        //get the wp number  
       headSerialReply(18);
       if (wp_no == 0) {
         lat = GPS_home[LAT];
         lon = GPS_home[LON];
       } else if (wp_no == 16) {
         lat = GPS_hold[LAT];
         lon = GPS_hold[LON];
       }
       serialize8(wp_no);
       serialize32(lat);
       serialize32(lon);
       serialize32(AltHold);           //altitude (cm) will come here -- temporary implementation to test feature with apps
       serialize16(0);                 //heading  will come here (deg)
       serialize16(0);                 //time to stay (ms) will come here 
       serialize8(0);                  //nav flag will come here
     }
     break;
   case MSP_SET_WP:
     {
       int32_t lat = 0,lon = 0,alt = 0;
       uint8_t wp_no = read8();        //get the wp number
       lat = read32();
       lon = read32();
       alt = read32();                 // to set altitude (cm)
       read16();                       // future: to set heading (deg)
       read16();                       // future: to set time to stay (ms)
       read8();                        // future: to set nav flag
       if (wp_no == 0) {
         GPS_home[LAT] = lat;
         GPS_home[LON] = lon;
         f.GPS_HOME_MODE = 0;          // with this flag, GPS_set_next_wp will be called in the next loop -- OK with SERIAL GPS / OK with I2C GPS
         f.GPS_FIX_HOME  = 1;
         if (alt != 0) AltHold = alt;  // temporary implementation to test feature with apps
       } else if (wp_no == 16) {       // OK with SERIAL GPS  --  NOK for I2C GPS / needs more code dev in order to inject GPS coord inside I2C GPS
         GPS_hold[LAT] = lat;
         GPS_hold[LON] = lon;
         if (alt != 0) AltHold = alt;  // temporary implementation to test feature with apps
         #if !defined(I2C_GPS)
           nav_mode      = NAV_MODE_WP;
           //GPS_set_next_wp(&GPS_hold[LAT],&GPS_hold[LON]);
         #endif
       }
     }
     headSerialReply(0);
     break;

   case MSP_RESET_CONF:
     if(!f.ARMED) LoadDefaults();
     headSerialReply(0);
     break;
   case MSP_ACC_CALIBRATION:
     if(!f.ARMED) calibratingA=512;
     headSerialReply(0);
     break;
   case MSP_MAG_CALIBRATION:
     if(!f.ARMED) f.CALIBRATE_MAG = 1;
     headSerialReply(0);
     break;

	 
   case MSP_EEPROM_WRITE:
     writeParams(0);
     headSerialReply(0);
     break;
   case MSP_DEBUG:
     s_struct((uint8_t*)&debug,8);
     break;
	
   default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
     headSerialError(0);
     break;
  }
  tailSerialReply();
}












