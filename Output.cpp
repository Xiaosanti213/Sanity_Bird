#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Alarms.h"

uint8_t PWM_PIN[8] = {9,10,11,3,6,5,A2,12};   


void writeMotors() { 
    OCR1A = motor[0]>>3; //  pin 9
    OCR1B = motor[1]>>3; //  pin 10
    OCR2A = motor[2]>>3; //  pin 11
    OCR2B = motor[3]>>3; //  pin 3
}


void writeAllMotors(int16_t mc) {   
// 给全部电机发送MINCOMMAND指令
  for (uint8_t i =0;i<NUMBER_MOTOR;i++) {
    motor[i]=mc;
  }
  writeMotors();//将舵量写入电机
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
  
  writeAllMotors(MINCOMMAND);
  delay(300);
}



void mixTable() {
  int16_t maxMotor;
  uint8_t i;
  #define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z
  motor[0] = PIDMIX(-1,+1,-1); //REAR_R
  motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
  motor[2] = PIDMIX(+1,+1,+1); //REAR_L
  motor[3] = PIDMIX(+1,-1,-1); //FRONT_L

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

      if (!f.ARMED)
        motor[i] = MINCOMMAND;
    }
}


















