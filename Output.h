#ifndef OUTPUT_H_
#define OUTPUT_H_

extern uint8_t PWM_PIN[8];
void initOutput();
void mixTable();

void writeMotorsServos();
#endif
