#include<RF24.h>
#include<SPI.h>
#include"printf.h"
#include<Servo.h>

#define CH2 5
#define CH9 10

Servo servo;
Servo motor;

unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

struct Data_Pack {
  byte Check;
  byte Speed;
  byte Angle;
}data;

RF24 radio(3, 2);
uint8_t address[][6] = {"txNode", "rxNode"};

void setup() {
  data.Check = 200;
  Serial.begin(115200);
  radio.begin();
  radio.openReadingPipe(0,address[0]);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
  printf_begin();
  radio.printDetails();
  pinMode(CH2,OUTPUT);
  pinMode(CH9,OUTPUT);
  servo.attach(CH2);
  motor.attach(CH9);
}

void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Pack)); 
    lastReceiveTime = millis(); // At this moment we have received the data
  }
  // Check whether we keep receving data, or we have a connection between the two modules
  currentTime = millis();
  if ( currentTime - lastReceiveTime > 1000 ) { // If current time is more then 1 second since we have recived the last data, that means we have lost connection
      resetData(); // If connection is lost, reset the data. It prevents unwanted behavior, for example if a drone has a throttle up and we lose connection, it can keep flying unless we reset the values
  }
  if(data.Check == 200){
    motor.writeMicroseconds(map(data.Speed,0,255,1000,2000));
    servo.write(map(data.Angle,0,255,45,135));
    Serial.print("RecSpeed:");
    Serial.println(map(data.Speed,0,255,1000,2000));
    Serial.print("RecAngle:");
    Serial.println(map(data.Angle,0,255,0,180));
  }
  
}

void resetData(){
   data.Speed = 0;
   data.Angle = 127;
}
