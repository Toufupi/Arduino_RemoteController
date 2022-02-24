#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2021-03-27 00:50:42

#include "Arduino.h"
#include "Arduino.h"
#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include "RS-FEC.h"
#include <Servo.h>
#include <EEPROM.h>

void updateRandChannel() ;
void updateNextChannel() ;
ISR(TIMER2_COMPA_vect) ;
int mapp(int val,int inL,int inH, int outL, int outH);
void setup() ;
void loop() ;
void calib_loop() ;

#include "tran.ino"


#endif
