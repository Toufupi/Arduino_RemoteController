#include<RF24.h>
#include<SPI.h>
#include"printf.h"

#define SPEED A2
#define ANGLE A1
//使用两个模拟值控制速度和方向

struct Data_Pack {
  byte Check;
  byte Speed;
  byte Angle;
}data;

//使用结构体储存信息，第一位为验证值，自行设定，防止多设备冲突

RF24 radio(7, 8);
uint8_t address[][6] = {"txNode", "rxNode"};

void setup() {
  data.Check = 200;
  Serial.begin(115200);
  radio.begin();
  radio.openWritingPipe(address[0]);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  printf_begin();
  radio.printDetails();
  pinMode(SPEED,INPUT);
  pinMode(ANGLE,INPUT);
}

void loop() {
  data.Speed = map(analogRead(SPEED), 0, 1023, 0, 255);
  data.Angle = map(analogRead(ANGLE), 0, 1023, 0, 255);
  Serial.print("SPEED:");
  Serial.println(data.Speed);
  Serial.print("ANGLE:");
  Serial.println(data.Angle);
  radio.write(&data,sizeof(Data_Pack));
  delay(1000);
}
