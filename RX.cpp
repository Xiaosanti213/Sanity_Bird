//现在使用普通接收机模式

#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "Serial.h"
#include "Protocol.h"
#include "MultiWii.h"
#include "Alarms.h"

volatile uint16_t rcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; 
// 间隔 [1000;2000]
  
static uint8_t rcChannel[RC_CHANS]  = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN,AUX2PIN,AUX3PIN,AUX4PIN};
static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {PCINT_RX_BITS}; 

void rxInt(void);



void configureReceiver() {
  /******************  给PCINT配置每一个接收机引脚  ***************************/
    // 激活PCINT引脚中断
    for(uint8_t i = 0; i < PCINT_PIN_COUNT; i++){ 
    // for循环用于初始化是可行的
      PCINT_RX_PORT |= PCInt_RX_Pins[i];
      PCINT_RX_MASK |= PCInt_RX_Pins[i];
    }
    PCICR = PCIR_PORT_BIT;
}



// 一个宏定义
#define RX_PIN_CHECK(pin_pos, rc_value_pos)                        \
    if (mask & PCInt_RX_Pins[pin_pos]) {                             \
      if (!(pin & PCInt_RX_Pins[pin_pos])) {                         \
        dTime = cTime-edgeTime[pin_pos];                             \
        if (900<dTime && dTime<2200) {                               \
          rcValue[rc_value_pos] = dTime;                             \
        }                                                            \
      } else edgeTime[pin_pos] = cTime;                              \
    }
	
	

// 端口改变中断触发服务函数
  ISR(RX_PC_INTERRUPT) { 
  //本中断服务对各个通道一致，引脚上会出现电位变化则触发中断
    uint8_t mask;
    uint8_t pin;
    uint16_t cTime,dTime;
    static uint16_t edgeTime[8];
    static uint8_t PCintLast;
  
    pin = RX_PCINT_PIN_PORT;
    // RX_PCINT_PIN_PORT 表示当前处理的数字引脚的电平状态
   
    mask = pin ^ PCintLast;   
    // doing a ^ 做亦或操作：当前中断和上次状态表示哪一个引脚出现变化
    cTime = micros();         
    // micros() 返回一个uint32_t数值, 我们只用其中16bits即够用
    sei();                    
    // 此时重新使能其他中断, 重置中断并非临界区，可以放心打断
    PCintLast = pin;          
    // 记录当前所有引脚的状态 PINs [D0-D7]
  
    // 下面这部分是说
    #if (PCINT_PIN_COUNT > 0)
      RX_PIN_CHECK(0,2);
    #endif
    #if (PCINT_PIN_COUNT > 1)
      RX_PIN_CHECK(1,4);
    #endif
    #if (PCINT_PIN_COUNT > 2)
      RX_PIN_CHECK(2,5);
    #endif
    #if (PCINT_PIN_COUNT > 3)
      RX_PIN_CHECK(3,6);
    #endif
    #if (PCINT_PIN_COUNT > 4)
      RX_PIN_CHECK(4,7);
    #endif
    #if (PCINT_PIN_COUNT > 5)
      RX_PIN_CHECK(5,0);
    #endif
    #if (PCINT_PIN_COUNT > 6)
      RX_PIN_CHECK(6,1);
    #endif
    #if (PCINT_PIN_COUNT > 7)
      RX_PIN_CHECK(7,3);
    #endif
    
  }	
	
	
	
	
uint16_t readRawRC(uint8_t chan) {
  uint16_t data;
  #if defined(SPEKTRUM)
    readSpektrum();
    if (chan < RC_CHANS) {
      data = rcValue[rcChannel[chan]];
    } else data = 1500;
  #else
    uint8_t oldSREG;
    oldSREG = SREG; cli();              // 压栈，禁用中断
    data = rcValue[rcChannel[chan]];    // 原子级别拷贝数据，防止被打断
    SREG = oldSREG;                     // 恢复中断前状态
  #endif
  return data; // 正确地返回禁用中断过程中拷贝的数据
}
	
	
	
	
	
	
	
void computeRC() {
  static uint16_t rcData4Values[RC_CHANS][4], rcDataMean[RC_CHANS];
  static uint8_t rc4ValuesIndex = 0;
  uint8_t chan,a;
  
    rc4ValuesIndex++;
    // 连续四次读取均值用的索引，在此处自增
    if (rc4ValuesIndex == 4) 
		rc4ValuesIndex = 0;
    // 对每一个通道执行下面的循环
    for (chan = 0; chan < RC_CHANS; chan++) {
      rcData4Values[chan][rc4ValuesIndex] = readRawRC(chan);
      
	  // 普通接收机PWM信号
      rcDataMean[chan] = 0;
      for (a=0;a<4;a++) 
          rcDataMean[chan] += rcData4Values[chan][a];
      rcDataMean[chan]= (rcDataMean[chan]+2)>>2;
      // 连续四次取均值加2是int计算时保证四舍五入
      if ( rcDataMean[chan] < (uint16_t)rcData[chan] -3)  
          rcData[chan] = rcDataMean[chan]+2;
      if ( rcDataMean[chan] > (uint16_t)rcData[chan] +3)  
          rcData[chan] = rcDataMean[chan]-2;
	  
	  /*
	  if (chan<8 && rcSerialCount > 0) { 
      // 来自于串口协议MSP的rcData覆盖RX数据直到计数器rcSerialCount到0
      // 这个推测每次调用函数时都会初始化计数变量，并且用于地面站发送数据
        rcSerialCount --;
        if (rcSerial[chan] >900) 
           {rcData[chan] = rcSerial[chan];} 
        // 只有相关通道被覆盖
      }
	  */
    }
}
		  
	
	
	

	
	
	
	
	
	
	
	
	
	
	
	
	




