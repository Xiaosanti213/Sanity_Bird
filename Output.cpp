#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Alarms.h"


uint8_t PWM_PIN[8] = {9,10,11,3,A0,A1,A2,12};   
// 前四个硬件PWM（1电机3舵机）+1个软件模拟PWM舵机
volatile uint8_t atomicServo[1] = {125};

void initializeServo();



// 直接向电机写入 比较寄存器的值
void writeMotorsServos() { 
    OCR1A = motor[0]>>3; //  pin 9
    OCR1B = motor[1]>>3; //  pin 10
    OCR2A = motor[2]>>3; //  pin 11
    OCR2B = motor[3]>>3; //  pin 3

    /*
    debug[0] = OCR1A;
    debug[1] = OCR1B;
    debug[2] = OCR2A;
    debug[3] = OCR2B;
    */
    uint8_t i;
    for(i = 0; i < 4; i++){
      atomicServo[i] = (servo[i]-1000)>>2;
      // 将[1000,2000]舵量映射到[0,250]
    }
	debug[0] = atomicServo[0];
}


void writeAllMotors(int16_t mc) {   
// 给全部电机发送MINCOMMAND指令
  for (uint8_t i =0;i<NUMBER_MOTOR;i++) {
    motor[i]=mc;
  }
  writeMotorsServos();//将舵量写入电机
}


void initOutput() {
  // 在setup()函数中调用，指定全部电机通道 输出模式
	for(uint8_t i=0;i<NUMBER_MOTOR;i++) {
    pinMode(PWM_PIN[i],OUTPUT);
  }
  TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
  TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
  TCCR2A |= _BV(COM2A1); // connect pin 11 to timer 2 channel A
  TCCR2A |= _BV(COM2B1); // connect pin 3 to timer 2 channel B


  // 电机写入最小指令
  writeAllMotors(MINCOMMAND);
  delay(300);

  // 初始化舵机
 initializeServo();
  
}




void initializeServo() {

  // 初始化舵机为输出模式 在def.h当中随promini而定
  // 这块初始化编号和后面指定输出编号不一致 是由于def中封装
  // 将A0用于模拟输入
  //SERVO_1_PINMODE;
  SERVO_2_PINMODE;
  SERVO_3_PINMODE;
  SERVO_4_PINMODE;


  // uses timer 0 Comperator A (8 bit)
  // 初始化计时器
  TCCR0A = 0; // normal counting mode
  TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
  // 计数到达比较值 进入中断
  #define SERVO_ISR TIMER0_COMPA_vect
  #define SERVO_CHANNEL OCR0A
  #define SERVO_1K_US 250
  #define SERVO_HALFK_US 125
  
}
  




// 封装宏: 前一个舵机信号线拉低->当前舵机信号线拉高->当前高电平延时1ms->再写入延时0~1ms舵量信号
#define SERVO_PULSE(PIN_HIGH,ACT_STATE,SERVO_NUM,LAST_PIN_LOW) \
    }else if(state == ACT_STATE){                                \
      LAST_PIN_LOW;                                              \
      PIN_HIGH;                                                  \
      SERVO_CHANNEL+=SERVO_1K_US;                                \
      state++;                                                   \
    }else if(state == ACT_STATE+1){                              \
      SERVO_CHANNEL+=atomicServo[SERVO_NUM];                     \
      state++;




// 计数器触发中断服务函数
ISR(SERVO_ISR) {
	static uint8_t state = 0; // indicates the current state of the chain
	if (state == 0) {
		SERVO_1_HIGH; // set servo 1's pin high 
		SERVO_CHANNEL += SERVO_1K_US; // wait 1000us
		state++; // count up the state
	}

	else if (state == 1) {
		SERVO_CHANNEL += atomicServo[SERVO_1_ARR_POS]; // (0-1000us)对应计数值为0-250 
		state++; // count up the state
		SERVO_PULSE(SERVO_2_HIGH,2,SERVO_2_ARR_POS,SERVO_1_LOW); // the same here
		SERVO_PULSE(SERVO_3_PIN_HIGH,4,SERVO_3_ARR_POS,SERVO_2_LOW);
		//SERVO_PULSE(SERVO_4_PIN_HIGH,6,SERVO_4_ARR_POS,SERVO_3_LOW);
		SERVO_3_LOW;
	}

	else {// 最后延时达到20ms周期
		SERVO_CHANNEL += SERVO_1K_US;
		if (state <= 40) {
			state += 2;
		}
		else {
			state = 0;
		}
	}
}





void mixTable() {
	int16_t maxMotor;
	uint8_t i;


	/*
	#define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z
	motor[0] = PIDMIX(-1,+1,-1); //REAR_R
	motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
	motor[2] = PIDMIX(+1,+1,+1); //REAR_L
	motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
	*/

  motor[0] = rcCommand[THROTTLE];
  motor[1] = rcCommand[ROLL];
  motor[2] = rcCommand[PITCH];
  motor[3] = rcCommand[YAW];

  /*
	//这块限制范围是防止波长超出限制导致反转
	maxMotor=motor[0];
	  for(i=1; i< NUMBER_MOTOR; i++)
		if (motor[i]>maxMotor)
		maxMotor=motor[i];//找到最大舵量的电机
	  for(i=0; i< NUMBER_MOTOR; i++) {
		if (maxMotor > MAXTHROTTLE)
		  motor[i] -= maxMotor - MAXTHROTTLE;//超出的部分减去
		motor[i] = constrain(motor[i], conf.minthrottle, MAXTHROTTLE);
		if ((rcData[THROTTLE] < MINCHECK))
		motor[i] = conf.minthrottle;
	  }
	 */ 

	  // 舵机输出舵量
		//if(f.PASSTHRU_MODE){   // Direct passthru from RX
	servo[0] = rcCommand[ROLL];                     //   Wing 1
	servo[1] = rcCommand[ROLL];                     //   Wing 2
	servo[2] = rcData[ROLL];				        //   Elevator
	//servo[3] = rcCommand[PITCH];                  //   Rudder
	debug[0] = rcCommand[ROLL];


  /*}else{
	// Assisted modes
	servo[0] = axisPID[ROLL];                    //   Wing 1
	servo[1] = axisPID[ROLL];                    //   Wing 2
	servo[2] = axisPID[YAW];                     //   Rudder
	servo[3] = axisPID[PITCH];                   //   Elevator
  }*/


  // 如果当前未解锁，设置油门最低

	if (!f.ARMED)
	{
		//for(uint8_t i = 0; i<4; i++)
		//{
		  //servo[i] = 125;//舵机中立点
		  //motor[i] = MINCOMMAND;
		//}
		motor[0] = MINCOMMAND;
		// 舵机不用解锁即可转 电机需要解锁手势 应该改成开关比较好
	}

}


















