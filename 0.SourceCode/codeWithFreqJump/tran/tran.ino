#include "Arduino.h"
#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include "RS-FEC.h"

#include <Servo.h>
#include <EEPROM.h>

#define STICK_ROLL 0
#define STICK_PITCH 1
#define STICK_YAW 3
#define STICK_THR 2
#define STICK_ROLL_REV_MAP 0, 255
#define STICK_PITCH_REV_MAP 255, 0
#define STICK_YAW_REV_MAP 255, 0
#define STICK_THR_REV_MAP 0, 255
#define SW_THR 2
#define LED_RF 6
#define LED_PWR 5

#define EEPROM_MAGIC_NUM 0x0726

#define RF24_PACKET_SIZE 8
#define RSFEC_LEN 4

RF24 radio(7, 8);
uint8_t address[][6] = { "recv", "tran" };

RS::ReedSolomon<RF24_PACKET_SIZE - RSFEC_LEN, RSFEC_LEN> rs;

struct RAW_RF {
	unsigned char rf_buf[RF24_PACKET_SIZE] = { };
} raw_rf;

struct RC_Payload {
	uint8_t roll;
	uint8_t pitch;
	uint8_t yaw;
	uint8_t thr;
	//uint8_t padding[2];
} rc_payload = { };
static_assert(sizeof(RC_Payload)==RF24_PACKET_SIZE-RSFEC_LEN,"RC_Payload size not match!");

struct RC_Feedback {
	float batt_v;
	//uint8_t padding[2];
} rc_feedback = { };
static_assert(sizeof(RC_Feedback)==RF24_PACKET_SIZE-RSFEC_LEN,"RC_Payload size not match!");

byte fhss_schema[] = { 113, 125, 96, 85, 72, 99, 91, 67 };
byte *fhss_ptr = fhss_schema;
const byte *fhs_end = fhss_schema + sizeof(fhss_schema) - 1;
void updateRandChannel() {
	randomSeed(analogRead(4));
	fhss_ptr = fhss_schema + random(sizeof(fhss_schema));
}
void updateNextChannel() {
	if (fhss_ptr == fhs_end) {
		fhss_ptr = fhss_schema;
	} else {
		++fhss_ptr;
	}
}

bool timer_event_fire = false;
bool twice_trgger = false;

ISR(TIMER2_COMPA_vect) {
	if (timer_event_fire && twice_trgger) {
		Serial.println("Schedule timeout!");
	}
	if (twice_trgger) {
		timer_event_fire = twice_trgger;
	}
	twice_trgger = !twice_trgger;
}

struct Stick_Calib {
	uint16_t roll[2] = { 512, 512 };
	uint16_t pitch[2] = { 512, 512 };
	uint16_t yaw[2] = { 512, 512 };
	uint16_t thr[2] = { 512, 512 };
	uint16_t magic = 0x0000;
} stick_calib = { };
byte *eeprom_ptr = (byte*) &stick_calib;

bool should_exit_calib = false;
void calib_loop();

int mapp(int val,int inL,int inH, int outL, int outH){
	val=val<inL?inL:val;
	val=val>inH?inH:val;
	return map(val,inL,inH,outL,outH);
}

uint64_t last_ts = 0;
void setup() {
	pinMode(SW_THR, INPUT);

	//analogReference(EXTERNAL);
	Serial.begin(115200);
	if (digitalRead(SW_THR) == HIGH) {
		Serial.println("Enter calibration mode...");
		while (true) {
			calib_loop();
			if (should_exit_calib) {
				break;
			}
		}
	}

	for (int i = 0; i < sizeof(stick_calib); ++i) {
		eeprom_ptr[i] = EEPROM.read(i);
	}

	if (stick_calib.magic != EEPROM_MAGIC_NUM) {
		Serial.println("NOT CALIBRATED!");
		while (true)
			;
	}

	printf_begin();	// RF24

	analogWrite(LED_RF, 0);
	analogWrite(LED_PWR, 0);
	analogWrite(LED_RF,255);

	while (!radio.begin()) {
		Serial.println(F("radio hardware is not responding!!"));
		delay(1000);
	}
	updateRandChannel();
	radio.setChannel(*fhss_ptr);
	radio.setPALevel(RF24_PA_MAX);
	radio.openWritingPipe(address[0]);
	radio.openReadingPipe(1, address[1]);
	radio.setAutoAck(false);
	//radio.disableCRC();
	radio.setDataRate(RF24_250KBPS);
	radio.setPayloadSize(RF24_PACKET_SIZE);
	radio.setRetries(0, 0);
	radio.powerUp();
	radio.startListening();
	radio.printDetails();

	cli();
	TCCR2A = 0;	// set entire TCCR2A register to 0
	TCCR2B = 0;	// same for TCCR2B
	TCNT2 = 0;	//initialize counter value to 0
	// set compare match register for 100hz increments
	OCR2A = 155;		// = (16*10^6) / (100*1024) - 1 (must be <256)
	// turn on CTC mode
	TCCR2A |= (1 << WGM21);
	// Set CS21 bit for 1024 prescaler
	TCCR2B |= (1 << CS20 | 1 << CS21 | 1 << CS22);
	// enable timer compare interrupt
	TIMSK2 |= (1 << OCIE2A);
	sei();

	last_ts = millis();
}

uint32_t rx_count = 0;
uint32_t loopcount = 0;
void loop() {
	if (radio.failureDetected) {
		Serial.println("hw failure");
	}

	loopcount++;
	if (timer_event_fire) {
		timer_event_fire = false;
		updateNextChannel();
		radio.setChannel(*fhss_ptr);
		rc_payload.roll = 128;//mapp(analogRead(STICK_ROLL), stick_calib.roll[0], stick_calib.roll[1], STICK_ROLL_REV_MAP);
		rc_payload.pitch = mapp(analogRead(STICK_PITCH), stick_calib.pitch[0], stick_calib.pitch[1],
		STICK_PITCH_REV_MAP);
		rc_payload.yaw = mapp(analogRead(STICK_YAW), stick_calib.yaw[0], stick_calib.yaw[1], STICK_YAW_REV_MAP);
		rc_payload.thr = 0;//mapp(analogRead(STICK_THR), stick_calib.thr[0], stick_calib.thr[1], STICK_THR_REV_MAP);
		/*Serial.print("roll ");Serial.print(rc_payload.roll);
		 Serial.print("pitch ");Serial.print(rc_payload.pitch);
		 Serial.print("yaw ");Serial.print(rc_payload.yaw);
		 Serial.print("thr ");Serial.println(rc_payload.thr);*/
		rs.Encode(&rc_payload, &raw_rf);
		radio.stopListening();
		radio.flush_tx();
		radio.writeFast(&raw_rf, RF24_PACKET_SIZE);
		radio.txStandBy();
		radio.startListening();
	}

	while (radio.available()) {
		radio.read(&raw_rf, RF24_PACKET_SIZE);
		if (rs.Decode(&raw_rf, &rc_feedback) == 0) {
			++rx_count;
			//Serial.print("batt: ");Serial.println(rc_feedback.batt_v);
		}
	}

	if (millis() - last_ts > 1000) {
		last_ts = millis();
		Serial.print("received count ");
		Serial.println(rx_count);
		Serial.print("batt: ");
		Serial.println(rc_feedback.batt_v);
		Serial.print("loop count ");
		Serial.println(loopcount);
		analogWrite(LED_RF,mapp(rx_count,0,50,0,255));
		if (rc_feedback.batt_v>3.6*3){
			analogWrite(LED_PWR,255);
		}else{
			analogWrite(LED_PWR,0);
		}
		rx_count = 0;
		loopcount = 0;
	}
}

void calib_loop() {
	uint16_t ain_roll = analogRead(STICK_ROLL);
	uint16_t ain_pitch = analogRead(STICK_PITCH);
	uint16_t ain_yaw = analogRead(STICK_YAW);
	uint16_t ain_thr = analogRead(STICK_THR);
	if (stick_calib.roll[0] > ain_roll) {
		stick_calib.roll[0] = ain_roll;
	}
	if (stick_calib.roll[1] < ain_roll) {
		stick_calib.roll[1] = ain_roll;
	}
	if (stick_calib.pitch[0] > ain_pitch) {
		stick_calib.pitch[0] = ain_pitch;
	}
	if (stick_calib.pitch[1] < ain_pitch) {
		stick_calib.pitch[1] = ain_pitch;
	}
	if (stick_calib.yaw[0] > ain_yaw) {
		stick_calib.yaw[0] = ain_yaw;
	}
	if (stick_calib.yaw[1] < ain_yaw) {
		stick_calib.yaw[1] = ain_yaw;
	}
	if (stick_calib.thr[0] > ain_thr) {
		stick_calib.thr[0] = ain_thr;
	}
	if (stick_calib.thr[1] < ain_thr) {
		stick_calib.thr[1] = ain_thr;
	}

	if (digitalRead(SW_THR) == LOW) {
		should_exit_calib = true;
		stick_calib.magic = EEPROM_MAGIC_NUM;
		for (int i = 0; i < sizeof(stick_calib); ++i) {
			EEPROM.update(i, eeprom_ptr[i]);
		}
		Serial.print("roll min=");
		Serial.print(stick_calib.roll[0]);
		Serial.print(" roll max=");
		Serial.println(stick_calib.roll[1]);
		Serial.print("pitch min=");
		Serial.print(stick_calib.pitch[0]);
		Serial.print(" pitch max=");
		Serial.println(stick_calib.pitch[1]);
		Serial.print("yaw min=");
		Serial.print(stick_calib.yaw[0]);
		Serial.print(" yaw max=");
		Serial.println(stick_calib.yaw[1]);
		Serial.print("thr min=");
		Serial.print(stick_calib.thr[0]);
		Serial.print(" thr max=");
		Serial.println(stick_calib.thr[1]);
	}
	delay(20);
	/*Serial.print(ain_roll);
	Serial.print(" ");
	Serial.print(ain_pitch);
	Serial.print(" ");
	Serial.print(ain_yaw);
	Serial.print(" ");
	Serial.print(ain_thr);
	Serial.println();*/
}
