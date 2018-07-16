#ifndef DEF_H_
#define DEF_H_



/**************************************************************************************/
/***************             Proc specific definitions             ********************/
/**************************************************************************************/
// Proc auto detection
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  #define PROMINI
#endif
#if defined(__AVR_ATmega32U4__) || defined(TEENSY20)
  #define PROMICRO
#endif
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
  #define MEGA
#endif


/**************************************************************************************/
/***************             motor and servo numbers               ********************/
/**************************************************************************************/
#define SERVO_RATES      {30,30,100,100,100,100,100,100}


#define SERVO
#define NUMBER_MOTOR     4
#define DYNBAL 0
#define FLAP 0
#define TRI_SERVO  6

#define PRI_SERVO_FROM   2 // use servo from 2 to 4
#define PRI_SERVO_TO     4

 
/**************************   atmega328P (Promini)  ************************************/
#if defined(PROMINI)
  #if !defined(MONGOOSE1_0)
    #define LEDPIN_PINMODE             pinMode (13, OUTPUT);
    #define LEDPIN_TOGGLE              PINB |= 1<<5;     //switch LEDPIN state (digital PIN 13)
    #define LEDPIN_OFF                 PORTB &= ~(1<<5);
    #define LEDPIN_ON                  PORTB |= (1<<5);
  #endif
  #if !defined(RCAUXPIN8) 
    #if !defined(MONGOOSE1_0)
      #define BUZZERPIN_PINMODE          pinMode (8, OUTPUT);
      #if NUMBER_MOTOR >4
        #undef PILOTLAMP
      #endif
      #if defined PILOTLAMP && NUMBER_MOTOR <5
        #define    PL_PIN_ON            PORTB |= 1;
        #define    PL_PIN_OFF           PORTB &= ~1;
      #else
        #define BUZZERPIN_ON            PORTB |= 1;
        #define BUZZERPIN_OFF           PORTB &= ~1;
      #endif 
    #endif
  #else
    #define BUZZERPIN_PINMODE          ;
    #define BUZZERPIN_ON               ;
    #define BUZZERPIN_OFF              ;
    #define RCAUXPIN
  #endif
  #if !defined(RCAUXPIN12) && !defined(DISABLE_POWER_PIN)
    #define POWERPIN_PINMODE           pinMode (12, OUTPUT);
    #define POWERPIN_ON                PORTB |= 1<<4;
    #define POWERPIN_OFF               PORTB &= ~(1<<4); //switch OFF WMP, digital PIN 12
  #else
    #define POWERPIN_PINMODE           ;
    #define POWERPIN_ON                ;
    #define POWERPIN_OFF               ;
  #endif
  #if defined(RCAUXPIN12)
    #define RCAUXPIN
  #endif
  #define I2C_PULLUPS_ENABLE         PORTC |= 1<<4; PORTC |= 1<<5;   // PIN A4&A5 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTC &= ~(1<<4); PORTC &= ~(1<<5);
  #if !defined(MONGOOSE1_0)
    #define PINMODE_LCD                pinMode(0, OUTPUT);
    #define LCDPIN_OFF                 PORTD &= ~1; //switch OFF digital PIN 0
    #define LCDPIN_ON                  PORTD |= 1;
    #define STABLEPIN_PINMODE          ;
    #define STABLEPIN_ON               ;
    #define STABLEPIN_OFF              ;
  #endif 
  #define PPM_PIN_INTERRUPT          attachInterrupt(0, rxInt, RISING); //PIN 0
  #define SPEK_SERIAL_PORT           0
  //RX PIN assignment inside the port //for PORTD
  #define THROTTLEPIN                2
  #define ROLLPIN                    4
  #define PITCHPIN                   5
  #define YAWPIN                     6
  #define AUX1PIN                    7
  #define AUX2PIN                    0 // optional PIN 8 or PIN 12
  #define AUX3PIN                    1 // unused 
  #define AUX4PIN                    3 // unused 
    
  #define PCINT_PIN_COUNT            5
  #define PCINT_RX_BITS              (1<<2),(1<<4),(1<<5),(1<<6),(1<<7)
  #define PCINT_RX_PORT              PORTD
  #define PCINT_RX_MASK              PCMSK2
  #define PCIR_PORT_BIT              (1<<2)
  #define RX_PC_INTERRUPT            PCINT2_vect
  #define RX_PCINT_PIN_PORT          PIND
  #define V_BATPIN                   A3    // Analog PIN 3
  #define PSENSORPIN                 A2    // Analog PIN 2
  
  #if defined(A0_A1_PIN_HEX) || (NUMBER_MOTOR > 6)
    #define SOFT_PWM_1_PIN_HIGH        PORTC |= 1<<0;
    #define SOFT_PWM_1_PIN_LOW         PORTC &= ~(1<<0);
    #define SOFT_PWM_2_PIN_HIGH        PORTC |= 1<<1;
    #define SOFT_PWM_2_PIN_LOW         PORTC &= ~(1<<1);  
  #else
    #define SOFT_PWM_1_PIN_HIGH        PORTD |= 1<<5;
    #define SOFT_PWM_1_PIN_LOW         PORTD &= ~(1<<5);
    #define SOFT_PWM_2_PIN_HIGH        PORTD |= 1<<6;
    #define SOFT_PWM_2_PIN_LOW         PORTD &= ~(1<<6);
  #endif
  #define SOFT_PWM_3_PIN_HIGH        PORTC |= 1<<2;
  #define SOFT_PWM_3_PIN_LOW         PORTC &= ~(1<<2);
  #define SOFT_PWM_4_PIN_HIGH        PORTB |= 1<<4;
  #define SOFT_PWM_4_PIN_LOW         PORTB &= ~(1<<4);
  
  #define SERVO_1_PINMODE            pinMode(A0,OUTPUT); // 
  #define SERVO_1_PIN_HIGH           PORTC |= 1<<0;
  #define SERVO_1_PIN_LOW            PORTC &= ~(1<<0);
  #define SERVO_2_PINMODE            pinMode(A1,OUTPUT); // 
  #define SERVO_2_PIN_HIGH           PORTC |= 1<<1;
  #define SERVO_2_PIN_LOW            PORTC &= ~(1<<1);
  #define SERVO_3_PINMODE            pinMode(A2,OUTPUT); // 
  #define SERVO_3_PIN_HIGH           PORTC |= 1<<2;
  #define SERVO_3_PIN_LOW            PORTC &= ~(1<<2);
  #if !defined(MONGOOSE1_0)
    #define SERVO_4_PINMODE            pinMode(12,OUTPUT); // 
    #define SERVO_4_PIN_HIGH           PORTB |= 1<<4;
    #define SERVO_4_PIN_LOW            PORTB &= ~(1<<4);
  #endif
  #define SERVO_5_PINMODE            pinMode(11,OUTPUT); // 
  #define SERVO_5_PIN_HIGH           PORTB |= 1<<3;
  #define SERVO_5_PIN_LOW            PORTB &= ~(1<<3);
  #define SERVO_6_PINMODE            pinMode(3,OUTPUT);  // 
  #define SERVO_6_PIN_HIGH           PORTD|= 1<<3;
  #define SERVO_6_PIN_LOW            PORTD &= ~(1<<3);
  #define SERVO_7_PINMODE            pinMode(10,OUTPUT); // 
  #define SERVO_7_PIN_HIGH           PORTB |= 1<<2;
  #define SERVO_7_PIN_LOW            PORTB &= ~(1<<2);
  #define SERVO_8_PINMODE            pinMode(9,OUTPUT); // 
  #define SERVO_8_PIN_HIGH           PORTB |= 1<<1;
  #define SERVO_8_PIN_LOW            PORTB &= ~(1<<1);
#endif




/**********************   Sort the Servos for the most ideal SW PWM     ************************/
// this define block sorts the above slected servos to be in a simple order from 1 - (count of total servos)
// its pretty fat but its the best way i found to get less compiled code and max speed in the ISR without loosing its flexibility
#if (PRI_SERVO_FROM == 1) || (SEC_SERVO_FROM == 1)
  #define LAST_LOW SERVO_1_PIN_LOW
  #define SERVO_1_HIGH SERVO_1_PIN_HIGH
  #define SERVO_1_LOW SERVO_1_PIN_LOW
  #define SERVO_1_ARR_POS  0
#endif
#if (PRI_SERVO_FROM <= 2 && PRI_SERVO_TO >= 2) || (SEC_SERVO_FROM <= 2 && SEC_SERVO_TO >= 2) 
  #undef LAST_LOW
  #define LAST_LOW SERVO_2_PIN_LOW
  #if !defined(SERVO_1_HIGH)
    #define SERVO_1_HIGH SERVO_2_PIN_HIGH
    #define SERVO_1_LOW SERVO_2_PIN_LOW  
    #define SERVO_1_ARR_POS 1
  #else
    #define SERVO_2_HIGH SERVO_2_PIN_HIGH
    #define SERVO_2_LOW SERVO_2_PIN_LOW   
    #define SERVO_2_ARR_POS 1
  #endif
#endif
#if (PRI_SERVO_FROM <= 3 && PRI_SERVO_TO >= 3) || (SEC_SERVO_FROM <= 3 && SEC_SERVO_TO >= 3) 
  #undef LAST_LOW
  #define LAST_LOW SERVO_3_PIN_LOW
  #if !defined(SERVO_1_HIGH)
    #define SERVO_1_HIGH SERVO_3_PIN_HIGH
    #define SERVO_1_LOW SERVO_3_PIN_LOW
    #define SERVO_1_ARR_POS 2 
  #elif !defined(SERVO_2_HIGH)
    #define SERVO_2_HIGH SERVO_3_PIN_HIGH
    #define SERVO_2_LOW SERVO_3_PIN_LOW 
    #define SERVO_2_ARR_POS 2 
  #else
    #define SERVO_3_HIGH SERVO_3_PIN_HIGH
    #define SERVO_3_LOW SERVO_3_PIN_LOW  
    #define SERVO_3_ARR_POS 2   
  #endif
#endif
#if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4) || (SEC_SERVO_FROM <= 4 && SEC_SERVO_TO >= 4) 
  #undef LAST_LOW
  #define LAST_LOW SERVO_4_PIN_LOW
  #if !defined(SERVO_1_HIGH)
    #define SERVO_1_HIGH SERVO_4_PIN_HIGH
    #define SERVO_1_LOW SERVO_4_PIN_LOW
    #define SERVO_1_ARR_POS 3  
  #elif !defined(SERVO_2_HIGH)
    #define SERVO_2_HIGH SERVO_4_PIN_HIGH
    #define SERVO_2_LOW SERVO_4_PIN_LOW
    #define SERVO_2_ARR_POS 3
  #elif !defined(SERVO_3_HIGH)
    #define SERVO_3_HIGH SERVO_4_PIN_HIGH
    #define SERVO_3_LOW SERVO_4_PIN_LOW
    #define SERVO_3_ARR_POS 3    
  #else
    #define SERVO_4_HIGH SERVO_4_PIN_HIGH
    #define SERVO_4_LOW SERVO_4_PIN_LOW 
    #define SERVO_4_ARR_POS 3     
  #endif
#endif
#if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5) || (SEC_SERVO_FROM <= 5 && SEC_SERVO_TO >= 5)
  #undef LAST_LOW
  #define LAST_LOW SERVO_5_PIN_LOW
  #if !defined(SERVO_1_HIGH)
    #define SERVO_1_HIGH SERVO_5_PIN_HIGH
    #define SERVO_1_LOW SERVO_5_PIN_LOW
    #define SERVO_1_ARR_POS 4   
  #elif !defined(SERVO_2_HIGH)
    #define SERVO_2_HIGH SERVO_5_PIN_HIGH
    #define SERVO_2_LOW SERVO_5_PIN_LOW
    #define SERVO_2_ARR_POS 4  
  #elif !defined(SERVO_3_HIGH)
    #define SERVO_3_HIGH SERVO_5_PIN_HIGH
    #define SERVO_3_LOW SERVO_5_PIN_LOW
    #define SERVO_3_ARR_POS 4   
  #elif !defined(SERVO_4_HIGH)
    #define SERVO_4_HIGH SERVO_5_PIN_HIGH
    #define SERVO_4_LOW SERVO_5_PIN_LOW
    #define SERVO_4_ARR_POS 4   
  #else
    #define SERVO_5_HIGH SERVO_5_PIN_HIGH
    #define SERVO_5_LOW SERVO_5_PIN_LOW 
    #define SERVO_5_ARR_POS 4     
  #endif
#endif
#if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6) || (SEC_SERVO_FROM <= 6 && SEC_SERVO_TO >= 6)
  #undef LAST_LOW
  #define LAST_LOW SERVO_6_PIN_LOW
  #if !defined(SERVO_1_HIGH)
    #define SERVO_1_HIGH SERVO_6_PIN_HIGH
    #define SERVO_1_LOW SERVO_6_PIN_LOW 
    #define SERVO_1_ARR_POS 5 
  #elif !defined(SERVO_2_HIGH)
    #define SERVO_2_HIGH SERVO_6_PIN_HIGH
    #define SERVO_2_LOW SERVO_6_PIN_LOW
    #define SERVO_2_ARR_POS 5 
  #elif !defined(SERVO_3_HIGH)
    #define SERVO_3_HIGH SERVO_6_PIN_HIGH
    #define SERVO_3_LOW SERVO_6_PIN_LOW
    #define SERVO_3_ARR_POS 5   
  #elif !defined(SERVO_4_HIGH)
    #define SERVO_4_HIGH SERVO_6_PIN_HIGH
    #define SERVO_4_LOW SERVO_6_PIN_LOW 
    #define SERVO_4_ARR_POS 5  
  #elif !defined(SERVO_5_HIGH)
    #define SERVO_5_HIGH SERVO_6_PIN_HIGH
    #define SERVO_5_LOW SERVO_6_PIN_LOW 
    #define SERVO_5_ARR_POS 5  
  #else
    #define SERVO_6_HIGH SERVO_6_PIN_HIGH
    #define SERVO_6_LOW SERVO_6_PIN_LOW  
    #define SERVO_6_ARR_POS 5   
  #endif
#endif
#if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7) || (SEC_SERVO_FROM <= 7 && SEC_SERVO_TO >= 7)
  #undef LAST_LOW
  #define LAST_LOW SERVO_7_PIN_LOW
  #if !defined(SERVO_1_HIGH)
    #define SERVO_1_HIGH SERVO_7_PIN_HIGH
    #define SERVO_1_LOW SERVO_7_PIN_LOW 
    #define SERVO_1_ARR_POS 6 
  #elif !defined(SERVO_2_HIGH)
    #define SERVO_2_HIGH SERVO_7_PIN_HIGH
    #define SERVO_2_LOW SERVO_7_PIN_LOW
    #define SERVO_2_ARR_POS 6 
  #elif !defined(SERVO_3_HIGH)
    #define SERVO_3_HIGH SERVO_7_PIN_HIGH
    #define SERVO_3_LOW SERVO_7_PIN_LOW
    #define SERVO_3_ARR_POS 6   
  #elif !defined(SERVO_4_HIGH)
    #define SERVO_4_HIGH SERVO_7_PIN_HIGH
    #define SERVO_4_LOW SERVO_7_PIN_LOW 
    #define SERVO_4_ARR_POS 6  
  #elif !defined(SERVO_5_HIGH)
    #define SERVO_5_HIGH SERVO_7_PIN_HIGH
    #define SERVO_5_LOW SERVO_7_PIN_LOW 
    #define SERVO_5_ARR_POS 6  
  #elif !defined(SERVO_6_HIGH)
    #define SERVO_6_HIGH SERVO_7_PIN_HIGH
    #define SERVO_6_LOW SERVO_7_PIN_LOW 
    #define SERVO_6_ARR_POS 6  
  #else
    #define SERVO_7_HIGH SERVO_7_PIN_HIGH
    #define SERVO_7_LOW SERVO_7_PIN_LOW  
    #define SERVO_7_ARR_POS 6   
  #endif
#endif
#if (PRI_SERVO_FROM <= 8 && PRI_SERVO_TO >= 8) || (SEC_SERVO_FROM <= 8 && SEC_SERVO_TO >= 8) 
  #undef LAST_LOW
  #define LAST_LOW SERVO_8_PIN_LOW
  #if !defined(SERVO_1_HIGH)
    #define SERVO_1_HIGH SERVO_8_PIN_HIGH
    #define SERVO_1_LOW SERVO_8_PIN_LOW 
    #define SERVO_1_ARR_POS 7 
  #elif !defined(SERVO_2_HIGH)
    #define SERVO_2_HIGH SERVO_8_PIN_HIGH
    #define SERVO_2_LOW SERVO_8_PIN_LOW
    #define SERVO_2_ARR_POS 7
  #elif !defined(SERVO_3_HIGH)
    #define SERVO_3_HIGH SERVO_8_PIN_HIGH
    #define SERVO_3_LOW SERVO_8_PIN_LOW
    #define SERVO_3_ARR_POS 7  
  #elif !defined(SERVO_4_HIGH)
    #define SERVO_4_HIGH SERVO_8_PIN_HIGH
    #define SERVO_4_LOW SERVO_8_PIN_LOW
    #define SERVO_4_ARR_POS 7  
  #elif !defined(SERVO_5_HIGH)
    #define SERVO_5_HIGH SERVO_8_PIN_HIGH
    #define SERVO_5_LOW SERVO_8_PIN_LOW 
    #define SERVO_5_ARR_POS 7  
  #elif !defined(SERVO_6_HIGH)
    #define SERVO_6_HIGH SERVO_8_PIN_HIGH
    #define SERVO_6_LOW SERVO_8_PIN_LOW 
    #define SERVO_6_ARR_POS 7 
  #elif !defined(SERVO_7_HIGH)
    #define SERVO_7_HIGH SERVO_8_PIN_HIGH
    #define SERVO_7_LOW SERVO_8_PIN_LOW 
    #define SERVO_7_ARR_POS 7  
  #else
    #define SERVO_8_HIGH SERVO_8_PIN_HIGH
    #define SERVO_8_LOW SERVO_8_PIN_LOW  
    #define SERVO_8_ARR_POS 7   
  #endif
#endif

#if ( defined(MEGA) && defined(MEGA_HW_PWM_SERVOS) ) || (defined(PROMICRO) && defined(A32U4_4_HW_PWM_SERVOS))
  #undef SERVO_1_HIGH                                    // No software PWM's if we use hardware MEGA PWM or promicro hardware pwm
  #define HW_PWM_SERVOS
#endif


/**************************************************************************************/
/***************      IMU Orientations and Sensor definitions      ********************/
/**************************************************************************************/


//板上传感器资源
#if defined(HK_MultiWii_SE_V2 ) 
  #define MPU6050 
  #define HMC5883 
  #define BMP085 
  #define ACC_ORIENTATION(X, Y, Z) {imu.accADC[ROLL]  = -X; imu.accADC[PITCH]  = -Y; imu.accADC[YAW]  =  Z;} 
  #define GYRO_ORIENTATION(X, Y, Z){imu.gyroADC[ROLL] =  Y; imu.gyroADC[PITCH] = -X; imu.gyroADC[YAW] = -Z;} 
  #define MAG_ORIENTATION(X, Y, Z) {imu.magADC[ROLL]  =  X; imu.magADC[PITCH]  =  Y; imu.magADC[YAW]  = -Z;} 
  #define MPU6050_EN_I2C_BYPASS // MAG connected to the AUX I2C bus of MPU6050 
  #undef INTERNAL_I2C_PULLUPS 
#endif



/**************************************************************************************/
/***************              Sensor Type definitions              ********************/
/**************************************************************************************/

#if defined(ADXL345) || defined(BMA020) || defined(BMA180) || defined(BMA280) || defined(NUNCHACK) || defined(MMA7455) || defined(ADCACC) || defined(LIS3LV02) || defined(LSM303DLx_ACC) || defined(MPU6050) || defined(LSM330) || defined(MMA8451Q) || defined(NUNCHUCK)
  #define ACC 1
#else
  #define ACC 0
#endif

#if defined(HMC5883) || defined(HMC5843) || defined(AK8975) || defined(MAG3110)
  #define MAG 1
#else
  #define MAG 0
#endif

#if defined(ITG3200) || defined(L3G4200D) || defined(MPU6050) || defined(LSM330) || defined(MPU3050) || defined(WMP)
  #define GYRO 1
#else
  #define GYRO 0
#endif

#if defined(BMP085) || defined(MS561101BA)
  #define BARO 1
#else
  #define BARO 0
#endif

#if defined(GPS_PROMINI_SERIAL) && defined(PROMINI)
  #define GPS_SERIAL 0
  #define GPS_PROMINI
#endif

#if defined(GPS_SERIAL)  || defined(I2C_GPS) || defined(GPS_FROM_OSD)
  #define GPS 1
#else
  #define GPS 0
#endif

#if defined(SRF02) || defined(SRF08) || defined(SRF10) || defined(SRC235) || defined(I2C_GPS_SONAR)
  #define SONAR 1
#else
  #define SONAR 0
#endif


#if defined(MMA7455)
  #define ACC_1G 64
#endif
#if defined(MMA8451Q)
  #define ACC_1G 512
#endif
#if defined(ADXL345)
  #define ACC_1G 265
#endif
#if defined(BMA180)
  #define ACC_1G 255
#endif
#if defined(BMA280)
  #define ACC_1G 255
#endif
#if defined(BMA020)
  #define ACC_1G 63
#endif
#if defined(NUNCHACK)
  #define ACC_1G 200
#endif
#if defined(LIS3LV02)
  #define ACC_1G 256
#endif
#if defined(LSM303DLx_ACC)
  #define ACC_1G 256
#endif
#if defined(ADCACC)
  #define ACC_1G 75
#endif
#if defined(MPU6050)
  #if defined(FREEIMUv04)
    #define ACC_1G 255
  #else
    #define ACC_1G 512
  #endif
#endif
#if defined(LSM330)
  #define ACC_1G 256
#endif
#if defined(NUNCHUCK)
  #define ACC_1G 200
#endif
#if !defined(ACC_1G)
  #define ACC_1G 256
#endif
#define ACCZ_25deg   (int16_t)(ACC_1G * 0.90631) // 0.90631 = cos(25deg) (cos(theta) of accZ comparison)
#define ACC_VelScale (9.80665f / 10000.0f / ACC_1G)

#if defined(ITG3200)
  #define GYRO_SCALE (4 / 14.375 * PI / 180.0 / 1000000.0) //ITG3200   14.375 LSB/(deg/s) and we ignore the last 2 bits
#endif
#if defined(L3G4200D)
  #define GYRO_SCALE ((4.0f * PI * 70.0f)/(1000.0f * 180.0f * 1000000.0f))
#endif
#if defined(MPU6050)
  #define GYRO_SCALE (4 / 16.4 * PI / 180.0 / 1000000.0)   //MPU6050 and MPU3050   16.4 LSB/(deg/s) and we ignore the last 2 bits
#endif
#if defined(LSM330)
  #define GYRO_SCALE ((4.0f * PI * 70.0f)/(1000.0f * 180.0f * 1000000.0f)) // like L3G4200D
#endif
#if defined(MPU3050)
  #define GYRO_SCALE (4 / 16.4 * PI / 180.0 / 1000000.0)   //MPU6050 and MPU3050   16.4 LSB/(deg/s) and we ignore the last 2 bits
#endif
#if defined(WMP)
  #define GYRO_SCALE (1.0f/200e6f)
#endif

/**************************************************************************************/
/***************      Multitype decleration for the GUI's          ********************/
/**************************************************************************************/
#if defined(TRI)
  #define MULTITYPE 1
#elif defined(QUADP)
  #define MULTITYPE 2
#elif defined(QUADX)
  #define MULTITYPE 3
#elif defined(BI)
  #define MULTITYPE 4
  #define SERVO_RATES      {30,30,100,100,0,1,100,100}
#elif defined(GIMBAL)
  #define MULTITYPE 5
#elif defined(Y6)
  #define MULTITYPE 6
#elif defined(HEX6)
  #define MULTITYPE 7
#elif defined(FLYING_WING)
  #define MULTITYPE 8
  #define SERVO_RATES      {30,30,100,0,1,100,100,100}
#elif defined(Y4)
  #define MULTITYPE 9
#elif defined(HEX6X)
  #define MULTITYPE 10
#elif defined(OCTOX8)
  #define MULTITYPE 11   //the JAVA GUI is the same for all 8 motor configs 
#elif defined(OCTOFLATP)
  #define MULTITYPE 12   //12  for MultiWinGui
#elif defined(OCTOFLATX)
  #define MULTITYPE 13   //13  for MultiWinGui 
#elif defined(AIRPLANE)
  #define MULTITYPE 14    
  #define SERVO_RATES      {30,30,100,100,-100,100,100,100}
#elif defined (HELI_120_CCPM)   
  #define MULTITYPE 15      
#elif defined (HELI_90_DEG)   
  #define MULTITYPE 16      
  #define SERVO_RATES      {30,30,100,-100,-100,100,100,100}
#elif defined(VTAIL4)
  #define MULTITYPE 17
#elif defined(HEX6H)
  #define MULTITYPE 18
#elif defined(SINGLECOPTER)
  #define MULTITYPE 20
  #define SERVO_RATES      {30,30,100,0,1,0,1,100}
#elif defined(DUALCOPTER)
  #define MULTITYPE 20
#endif

/**************************************************************************************/
/***************          Some unsorted "chain" defines            ********************/
/**************************************************************************************/

#if defined (AIRPLANE) || defined(HELICOPTER)|| defined(SINGLECOPTER)|| defined(DUALCOPTER) && defined(PROMINI) 
  #if defined(D12_POWER)
    #define SERVO_4_PINMODE            ;  // D12
    #define SERVO_4_PIN_HIGH           ;
    #define SERVO_4_PIN_LOW            ;
  #else
    #undef POWERPIN_PINMODE
    #undef POWERPIN_ON
    #undef POWERPIN_OFF
    #define POWERPIN_PINMODE           ;
    #define POWERPIN_ON                ;
    #define POWERPIN_OFF               ;
  #endif
#endif

#if defined(POWERMETER_HARD) || defined(POWERMETER_SOFT)
  #define POWERMETER
  #define PLEVELSCALE 50 // step size you can use to set alarm
  #define PLEVELDIVSOFT 100000
  #define PLEVELDIV 36000
#endif

#if defined PILOTLAMP 
  #define    PL_CHANNEL OCR0B  //use B since A can be used by camstab
  #define    PL_ISR TIMER0_COMPB_vect
  #define    PL_INIT   TCCR0A=0;TIMSK0|=(1<<OCIE0B);PL_CHANNEL=PL_IDLE;PilotLamp(PL_GRN_OFF);PilotLamp(PL_BLU_OFF);PilotLamp(PL_RED_OFF);PilotLamp(PL_BZR_OFF);
  #define    BUZZERPIN_ON PilotLamp(PL_BZR_ON);
  #define    BUZZERPIN_OFF PilotLamp(PL_BZR_OFF);
  #define    PL_GRN_ON    25    // 100us
  #define    PL_GRN_OFF   50    // 200us
  #define    PL_BLU_ON    75    // 300us
  #define    PL_BLU_OFF   100    // 400us
  #define    PL_RED_ON    125    // 500us
  #define    PL_RED_OFF   150    // 600us
  #define    PL_BZR_ON    175    // 700us
  #define    PL_BZR_OFF   200    // 800us
  #define    PL_IDLE      125    // 100us
#endif

#if defined(PILOTLAMP)
  #define BUZZER
#endif

//all new Special RX's must be added here
//this is to avoid confusion :)
#if !defined(SERIAL_SUM_PPM) && !defined(SPEKTRUM) && !defined(SBUS)
  #define STANDARD_RX
#endif

// Spektrum Satellite
#if defined(SPEKTRUM)
  #define SPEK_FRAME_SIZE 16
  #if (SPEKTRUM == 1024)
    #define SPEK_CHAN_SHIFT  2       // Assumes 10 bit frames, that is 1024 mode.
    #define SPEK_CHAN_MASK   0x03    // Assumes 10 bit frames, that is 1024 mode.
    #define SPEK_DATA_SHIFT          // Assumes 10 bit frames, that is 1024 mode.
    #define SPEK_BIND_PULSES 3
  #endif
  #if (SPEKTRUM == 2048)
    #define SPEK_CHAN_SHIFT  3       // Assumes 11 bit frames, that is 2048 mode.
    #define SPEK_CHAN_MASK   0x07    // Assumes 11 bit frames, that is 2048 mode.
    #define SPEK_DATA_SHIFT >> 1     // Assumes 11 bit frames, that is 2048 mode.
    #define SPEK_BIND_PULSES 5
  #endif
  #if defined(SPEK_BIND)
    #if !defined(SPEK_BIND_GROUND)
      #define SPEK_BIND_GROUND 4
    #endif  
    #if !defined(SPEK_BIND_POWER)
      #define SPEK_BIND_POWER  5
    #endif  
    #if !defined(SPEK_BIND_DATA)
      #define SPEK_BIND_DATA   6
    #endif  
  #endif
#endif

#if defined(SBUS)
  #define RC_CHANS 18
#elif defined(SPEKTRUM) || defined(SERIAL_SUM_PPM)
  #define RC_CHANS 12
#else
  #define RC_CHANS 8
#endif


/**************************************************************************************/
/***************                       I2C GPS                     ********************/
/**************************************************************************************/
#if defined(I2C_GPS)
  #define I2C_GPS_ADDRESS                         0x20 //7 bits       
///////////////////////////////////////////////////////////////////////////////////////////////////
// I2C GPS NAV registers
///////////////////////////////////////////////////////////////////////////////////////////////////
//
#define I2C_GPS_STATUS_00                            00 //(Read only)
        #define I2C_GPS_STATUS_NEW_DATA       0x01      // New data is available (after every GGA frame)
        #define I2C_GPS_STATUS_2DFIX          0x02      // 2dfix achieved
        #define I2C_GPS_STATUS_3DFIX          0x04      // 3dfix achieved
        #define I2C_GPS_STATUS_WP_REACHED     0x08      // Active waypoint has been reached (not cleared until new waypoint is set)
        #define I2C_GPS_STATUS_NUMSATS        0xF0      // Number of sats in view

#define I2C_GPS_COMMAND                              01 // (write only)
        #define I2C_GPS_COMMAND_POSHOLD       0x01      // Start position hold at the current gps positon
        #define I2C_GPS_COMMAND_START_NAV     0x02      // get the WP from the command and start navigating toward it
        #define I2C_GPS_COMMAND_SET_WP        0x03      // copy current position to given WP      
        #define I2C_GPS_COMMAND_UPDATE_PIDS   0x04      // update PI and PID controllers from the PID registers, this must be called after a pid register is changed
        #define I2C_GPS_COMMAND_NAV_OVERRIDE  0x05      // do not nav since we tring to controll the copter manually (not implemented yet)
        #define I2C_GPS_COMMAND_STOP_NAV      0x06      // Stop navigation (zeroes out nav_lat and nav_lon
        #define I2C_GPS_COMMAND__7            0x07
        #define I2C_GPS_COMMAND__8            0x08      
        #define I2C_GPS_COMMAND__9            0x09
        #define I2C_GPS_COMMAND__a            0x0a
        #define I2C_GPS_COMMAND__b            0x0b
        #define I2C_GPS_COMMAND__c            0x0c
        #define I2C_GPS_COMMAND__d            0x0d
        #define I2C_GPS_COMMAND__e            0x0e
        #define I2C_GPS_COMMAND__f            0x0f

        #define I2C_GPS_COMMAND_WP_MASK       0xF0       // Waypoint number

#define I2C_GPS_WP_REG                              02   // Waypoint register (Read only)
        #define I2C_GPS_WP_REG_ACTIVE_MASK    0x0F       // Active Waypoint lower 4 bits
        #define I2C_GPS_WP_REG_PERVIOUS_MASK  0xF0       // pervious Waypoint upper 4 bits
        
#define I2C_GPS_REG_VERSION                         03   // Version of the I2C_NAV SW uint8_t
#define I2C_GPS_REG_RES2                            04   // reserved for future use (uint8_t)
#define I2C_GPS_REG_RES3                            05   // reserved for future use (uint8_t)
#define I2C_GPS_REG_RES4                            06   // reserved for future use (uint8_t)


#define I2C_GPS_LOCATION                            07   // current location 8 byte (lat, lon) int32_t
#define I2C_GPS_NAV_LAT                             15   // Desired banking towards north/south int16_t
#define I2C_GPS_NAV_LON                             17   // Desired banking toward east/west    int16_t
#define I2C_GPS_WP_DISTANCE                         19   // Distance to current WP in cm uint32
#define I2C_GPS_WP_TARGET_BEARING                   23   // bearing towards current wp 1deg = 1000 int16_t
#define I2C_GPS_NAV_BEARING                         25   // crosstrack corrected bearing towards current wp 1deg = 1000 int16_t
#define I2C_GPS_HOME_TO_COPTER_BEARING              27   // bearing from home to copter 1deg = 1000 int16_t
#define I2C_GPS_DISTANCE_TO_HOME                    29   // distance to home in m int16_t
        
#define I2C_GPS_GROUND_SPEED                        31   // GPS ground speed in m/s*100 (uint16_t)      (Read Only)
#define I2C_GPS_ALTITUDE                            33   // GPS altitude in meters (uint16_t)           (Read Only)
#define I2C_GPS_GROUND_COURSE                       35   // GPS ground course (uint16_t)
#define I2C_GPS_RES1                                37   // reserved for future use (uint16_t)
#define I2C_GPS_TIME                                39   // UTC Time from GPS in hhmmss.sss * 100 (uint32_t)(unneccesary precision) (Read Only)

//Writeable registers from here

#define I2C_GPS_CROSSTRACK_GAIN                     43    // Crosstrack gain *100 (1 - 0.01 100 - 1) uint8_t
#define I2C_GPS_SPEED_MIN                           44    // Minimum navigation speed cm/s uint8_t
#define I2C_GPS_SPEED_MAX                           45    // Maximum navigation speed cm/s uint16_t
#define I2C_GPS_RESERVED                            47    // Reserved for future use
#define I2C_GPS_WP_RADIUS                           49    // Radius of the wp in cm, within this radius we consider the wp reached (uint16_t)

#define I2C_GPS_NAV_FLAGS                           51    // Controls various functions of the I2C-GPS-NAV module
        #define I2C_NAV_FLAG_GPS_FILTER          0x80     // If this bit set GPS coordinates are filtered via a 5 element moving average filter
        #define I2C_NAV_FLAG_LOW_SPEED_D_FILTER  0x40     // If speed below .5m/s ignore D term in POSHOLD_RATE, this supposed to filter out noise

#define I2C_GPS_HOLD_P                              52    // poshold_P  *100 uint16_t
#define I2C_GPS_HOLD_I                              53    // poshold_I  *100 uint16_t
#define I2C_GPS_HOLD_IMAX                           54    // poshold_IMAX *1 uint8_t

#define I2C_GPS_HOLD_RATE_P                         55    // poshold_rate_P  *10 uint16_t
#define I2C_GPS_HOLD_RATE_I                         56    // poshold_rate_I  *100 uint16_t
#define I2C_GPS_HOLD_RATE_D                         57    // poshold_rate_D  *1000 uint16_t
#define I2C_GPS_HOLD_RATE_IMAX                      58    // poshold_rate_IMAX *1 uint8_t

#define I2C_GPS_NAV_P                               59    // nav_P  *10 uint16_t
#define I2C_GPS_NAV_I                               60    // nav_I  *100 uint16_t
#define I2C_GPS_NAV_D                               61    // nav_D  *1000 uint16_t
#define I2C_GPS_NAV_IMAX                            62    // nav_IMAX *1 uint8_t

#define I2C_GPS_WP0                                 63   //Waypoint 0 used for RTH location      (R/W)
#define I2C_GPS_WP1                                 74
#define I2C_GPS_WP2                                 85
#define I2C_GPS_WP3                                 96
#define I2C_GPS_WP4                                 107
#define I2C_GPS_WP5                                 118
#define I2C_GPS_WP6                                 129
#define I2C_GPS_WP7                                 140
#define I2C_GPS_WP8                                 151
#define I2C_GPS_WP9                                 162
#define I2C_GPS_WP10                                173
#define I2C_GPS_WP11                                184
#define I2C_GPS_WP12                                195
#define I2C_GPS_WP13                                206
#define I2C_GPS_WP14                                217
#define I2C_GPS_WP15                                228

#define I2C_GPS_SONAR_ALT                           239   // Sonar Altitude
///////////////////////////////////////////////////////////////////////////////////////////////////
// End register definition 
///////////////////////////////////////////////////////////////////////////////////////////////////

#endif

#if !(defined(DISPLAY_2LINES)) && !(defined(DISPLAY_MULTILINE))
  #if (defined(LCD_VT100)) || (defined(OLED_I2C_128x64) || defined(OLED_DIGOLE) )
    #define DISPLAY_MULTILINE
  #else
    #define DISPLAY_2LINES
  #endif
#endif

#if (defined(LCD_VT100))
  #if !(defined(MULTILINE_PRE))
    #define MULTILINE_PRE 6
  #endif
  #if !(defined(MULTILINE_POST))
    #define MULTILINE_POST 9
  #endif
  #if !(defined(DISPLAY_COLUMNS))
    #define DISPLAY_COLUMNS 40
  #endif
#elif (defined(OLED_I2C_128x64) && defined(DISPLAY_FONT_DSIZE))
  #if !(defined(MULTILINE_PRE))
    #define MULTILINE_PRE 1
  #endif
  #if !(defined(MULTILINE_POST))
    #define MULTILINE_POST 3
  #endif
  #if !(defined(DISPLAY_COLUMNS))
    #define DISPLAY_COLUMNS 21
  #endif
#elif (defined(OLED_I2C_128x64))
  #if !(defined(MULTILINE_PRE))
    #define MULTILINE_PRE 3
  #endif
  #if !(defined(MULTILINE_POST))
    #define MULTILINE_POST 5
  #endif
  #if !(defined(DISPLAY_COLUMNS))
    #define DISPLAY_COLUMNS 21
  #endif
#elif (defined(OLED_DIGOLE) && defined(DISPLAY_FONT_DSIZE))
  #if !(defined(MULTILINE_PRE))
    #define MULTILINE_PRE 2
  #endif
  #if !(defined(MULTILINE_POST))
    #define MULTILINE_POST 3
  #endif
#elif (defined(OLED_DIGOLE))
  #if !(defined(MULTILINE_PRE))
    #define MULTILINE_PRE 3
  #endif
  #if !(defined(MULTILINE_POST))
    #define MULTILINE_POST 4
  #endif
  #if !(defined(DISPLAY_COLUMNS))
    #define DISPLAY_COLUMNS 21
  #endif
#endif

#if !(defined(DISPLAY_COLUMNS))
  #define DISPLAY_COLUMNS 16
#endif


/**************************************************************************************/
/***************               override defaults                   ********************/
/**************************************************************************************/

  /***************               pin assignments ?  ********************/
  #ifdef OVERRIDE_V_BATPIN
    #undef V_BATPIN
    #define V_BATPIN OVERRIDE_V_BATPIN
  #endif
  #ifdef OVERRIDE_PSENSORPIN
    #undef PSENSORPIN
    #define PSENSORPIN OVERRIDE_PSENSORPIN
  #endif
  #ifdef OVERRIDE_LEDPIN_PINMODE
    #undef LEDPIN_PINMODE
    #undef LEDPIN_TOGGLE
    #undef LEDPIN_OFF
    #undef LEDPIN_ON
    #define LEDPIN_PINMODE OVERRIDE_LEDPIN_PINMODE
    #define LEDPIN_TOGGLE  OVERRIDE_LEDPIN_TOGGLE
    #define LEDPIN_OFF     OVERRIDE_LEDPIN_OFF
    #define LEDPIN_ON      OVERRIDE_LEDPIN_ON
  #endif
  #ifdef OVERRIDE_BUZZERPIN_PINMODE
    #undef BUZZERPIN_PINMODE
    #undef BUZZERPIN_ON
    #undef BUZZERPIN_OFF
    #define BUZZERPIN_PINMODE OVERRIDE_BUZZERPIN_PINMODE
    #define BUZZERPIN_ON      OVERRIDE_BUZZERPIN_ON
    #define BUZZERPIN_OFF     OVERRIDE_BUZZERPIN_OFF
  #endif

  /*********  sensors orientation - possibly overriding board defaults  *****/
  #ifdef FORCE_GYRO_ORIENTATION
    #undef GYRO_ORIENTATION
    #define GYRO_ORIENTATION FORCE_GYRO_ORIENTATION
  #endif
  #ifdef FORCE_ACC_ORIENTATION
    #undef ACC_ORIENTATION
    #define ACC_ORIENTATION FORCE_ACC_ORIENTATION
  #endif
  #ifdef FORCE_MAG_ORIENTATION
    #undef MAG_ORIENTATION
    #define MAG_ORIENTATION FORCE_MAG_ORIENTATION
  #endif

  /*********  servo rates                                               *****/
  #ifdef FORCE_SERVO_RATES
    #undef SERVO_RATES
    #define SERVO_RATES FORCE_SERVO_RATES
  #endif
/**************************************************************************************/
/***************               Error Checking Section              ********************/
/**************************************************************************************/

#ifndef NUMBER_MOTOR
        #error "NUMBER_MOTOR is not set, most likely you have not defined any type of multicopter"
#endif

#if (defined(LCD_DUMMY) || defined(LCD_SERIAL3W) || defined(LCD_TEXTSTAR) || defined(LCD_VT100) || defined(LCD_TTY) || defined(LCD_ETPP) || defined(LCD_LCD03) || defined(OLED_I2C_128x64) ) || defined(OLED_DIGOLE)
  #define HAS_LCD
#endif

#if (defined(LCD_CONF) || defined(LCD_TELEMETRY)) && !(defined(HAS_LCD) )
  #error "LCD_CONF or LCD_TELEMETRY defined, and choice of LCD not defined.  Uncomment one of LCD_SERIAL3W, LCD_TEXTSTAR, LCD_VT100, LCD_TTY or LCD_ETPP, LCD_LCD03, OLED_I2C_128x64, OLED_DIGOLE"
#endif

#if defined(POWERMETER_SOFT) && !(defined(VBAT))
        #error "to use powermeter, you must also define and configure VBAT"
#endif

#if defined(LCD_TELEMETRY_AUTO) && !(defined(LCD_TELEMETRY))
        #error "to use automatic telemetry, you MUST also define and configure LCD_TELEMETRY"
#endif

#if defined(LCD_TELEMETRY_STEP) && !(defined(LCD_TELEMETRY))
        #error "to use single step telemetry, you MUST also define and configure LCD_TELEMETRY"
#endif

#if defined(A32U4_4_HW_PWM_SERVOS) && !(defined(HELI_120_CCPM))
  #error "for your protection: A32U4_4_HW_PWM_SERVOS was not tested with your coptertype"
#endif

#endif /* DEF_H_ */
