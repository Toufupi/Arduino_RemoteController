#include "Arduino.h"
#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include "RS-FEC.h"

#include <Servo.h>

//#define USE_TIMER_2     true
//#include "TimerInterrupt.h"

#define BATTERY_VIN 0
#define BATTERY_V_DIV 3	// 10k+20k
#define MOTOR_OUT 5
#define MOTOR_LIMIT 2000
#define SERVO_OUT 6
#define SERVO_MIDDLE 1500
#define PWR_LED 3

#define RF24_PACKET_SIZE 8
#define RSFEC_LEN 4

#define SCHEDULE_INTERVAL_MS 10

#define MIXER_THR_TO_YAW 0.05f

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
} rc_payload;
static_assert(sizeof(RC_Payload)==RF24_PACKET_SIZE-RSFEC_LEN,"RC_Payload too big!");

struct RC_Feedback {
	float batt_v;
	//uint8_t padding[2];
} rc_feedback;
static_assert(sizeof(RC_Feedback)==RF24_PACKET_SIZE-RSFEC_LEN,"RC_Payload too big!");

byte fhss_schema[] = { 113, 125, 96, 85, 72, 99, 91, 67 };
uint8_t fhss_failed_count[sizeof(fhss_schema)] = { };
byte *fhss_ptr = fhss_schema;
const byte *fhs_end = fhss_schema + sizeof(fhss_schema) - 1;
bool trigger_fhss_lost = false;
uint8_t fhss_missed_slots = 0;
uint8_t fhss_missed_stopwait = 0;
#define FHSS_OUT_OF_SYNC_COUNT (50)
void updateRandChannel() {
	randomSeed(analogRead(1));
	fhss_ptr = fhss_schema + random(sizeof(fhss_schema));
}
void updateNextChannel() {
	if (fhss_ptr == fhs_end) {
		fhss_ptr = fhss_schema;
	} else {
		++fhss_ptr;
	}
}

Servo motor;
Servo servo;

uint16_t output_pwm_motor = 0;
uint16_t target_pwm_motor = 0;
uint16_t output_pwm_servo = 0;
uint16_t target_pwm_servo = 0;

bool timer_event_fire = false;
bool timerEventTypeHop = false;	// true: do hop; false: do event

ISR(TIMER2_COMPA_vect) {
	if (timer_event_fire && !timerEventTypeHop) {
		Serial.println("Schedule timeout!");
	}
	timerEventTypeHop = !timerEventTypeHop;
	if (timerEventTypeHop) {
		timer_event_fire = true;
	}
}

void setup_nrf() {
	while (!radio.begin()) {
		Serial.println(F("radio hardware is not responding!!"));
		delay(1000);
	}
	updateRandChannel();
	radio.setChannel(*fhss_ptr);
	radio.setPALevel(RF24_PA_MAX);
	radio.openWritingPipe(address[1]);
	radio.openReadingPipe(1, address[0]);
	radio.setAutoAck(false);
	//radio.disableCRC();
	radio.setDataRate(RF24_250KBPS);
	radio.setPayloadSize(RF24_PACKET_SIZE);
	radio.printDetails();

	radio.startListening();
}

void setup() {
	Serial.begin(115200);
	printf_begin();	// RF24

	setup_nrf();

	pinMode(PWR_LED, OUTPUT);
	analogWrite(PWR_LED, 0);
	motor.attach(MOTOR_OUT);
	servo.attach(SERVO_OUT);
	motor.writeMicroseconds(0);
	servo.writeMicroseconds(0);

	//ITimer2.init();
	//if (not ITimer2.attachInterruptInterval(SCHEDULE_INTERVAL_MS,
	//		Timer2Handler)) {
	//	Serial.println("Timer2 failed to start");
	//}
	cli();
	TCCR2A = 0;	// set entire TCCR2A register to 0
	TCCR2B = 0;	// same for TCCR2B
	TCNT2 = 0;	//initialize counter value to 0
	// set compare match register for 100hz increments
	OCR2A = 155;	// = (16*10^6) / (100*1024) - 1 (must be <256)
	// turn on CTC mode
	TCCR2A |= (1 << WGM21);
	// Set CS21 bit for 1024 prescaler
	TCCR2B |= (1 << CS20 | 1 << CS21 | 1 << CS22);
	// enable timer compare interrupt
	TIMSK2 |= (1 << OCIE2A);
	sei();
}

uint64_t last_ts;
uint32_t rx_count = 0;
void loop() {
	if (radio.failureDetected) {
		Serial.println("hw failure");
		setup_nrf();
	}

	if (timer_event_fire) {
		timer_event_fire = false;

		motor.writeMicroseconds(output_pwm_motor);
		servo.writeMicroseconds(output_pwm_servo);

		/*Serial.print("target ");
		 Serial.print(target_pwm_motor);Serial.print(" ");
		 Serial.print(target_pwm_servo);Serial.println();*/
		/*Serial.print("output ");
		 Serial.print(output_pwm_motor);Serial.print(" ");
		 Serial.print(output_pwm_servo);Serial.println();*/

		if (target_pwm_motor >= 800 && output_pwm_motor >= 800) {
			if ((int) (target_pwm_motor - output_pwm_motor) > 15) {
				output_pwm_motor += 10;
			} else {
				output_pwm_motor = target_pwm_motor;
			}
		} else if (target_pwm_motor >= 800) {
			output_pwm_motor = 1000;
		} else {
			output_pwm_motor = 0;
		}

		output_pwm_servo = target_pwm_servo;

		if (fhss_missed_slots == FHSS_OUT_OF_SYNC_COUNT) {// lots of slot missed, then listen on a random slot
			updateRandChannel();
			radio.setChannel(*fhss_ptr);
			trigger_fhss_lost = true;
			++fhss_missed_slots;	// bigger 1, so ISR will do nothing
		} else if (fhss_missed_slots < FHSS_OUT_OF_SYNC_COUNT) {// not treated as out-of-sync, hop and increase.
			++fhss_failed_count[fhss_ptr - fhss_schema];
			updateNextChannel();
			radio.setChannel(*fhss_ptr);
			++fhss_missed_slots;
		} else {
			++fhss_failed_count[fhss_ptr - fhss_schema];
			++fhss_missed_stopwait;
			if (fhss_missed_stopwait > FHSS_OUT_OF_SYNC_COUNT) {
				fhss_missed_stopwait = 0;
				updateRandChannel();
				radio.setChannel(*fhss_ptr);
			}
		}
	}

	while (radio.available()) {
		radio.read(&raw_rf, RF24_PACKET_SIZE);
		if (rs.Decode(&raw_rf, &rc_payload) == 0) {	// once sync
			//ITimer2.restartTimer();	// sync timer
			//ITimer2.setCount(0);
			TIMSK2 &= ~(1 << OCIE2A);
			TCNT2 = 0;
			TIMSK2 |= (1 << OCIE2A);
			fhss_missed_slots = 0;	// clear value
			fhss_missed_stopwait = 0;	// too
			timerEventTypeHop = false;	// next time it will be true, do hopping
			/*Serial.print("roll: ");
			 Serial.print(rc_payload.roll);
			 Serial.print("pitch: ");
			 Serial.print(rc_payload.pitch);
			 Serial.print("yaw: ");
			 Serial.print(rc_payload.yaw);
			 Serial.print("thr: ");
			 Serial.println(rc_payload.thr);*/
			//motor.writeMicroseconds(map(rc_payload.thr,0,255,1000,2000));
			//servo.writeMicroseconds(map(rc_payload.roll,0,255,1000,2000));
			target_pwm_motor = map(rc_payload.thr, 0, 255, 1000, 2000);
			target_pwm_motor =
					target_pwm_motor > MOTOR_LIMIT ?
					MOTOR_LIMIT :
														target_pwm_motor;
			int rc_yaw_val = (int) rc_payload.roll
					+ int(rc_payload.thr * MIXER_THR_TO_YAW);
			if (rc_yaw_val > 255)
				rc_yaw_val = 255;
			if (rc_yaw_val < 0)
				rc_yaw_val = 0;
			target_pwm_servo = map(rc_yaw_val, 0, 255, 1000, 2000);
			++rx_count;
			if (fhss_failed_count[fhss_ptr - fhss_schema] != 0) {
				--fhss_failed_count[fhss_ptr - fhss_schema];
			}

			rc_feedback.batt_v = 1.0 * analogRead(0) / 1024 * BATTERY_V_DIV * 5;
			rs.Encode(&rc_feedback, &raw_rf);
			radio.stopListening();
			radio.writeFast(&raw_rf, RF24_PACKET_SIZE);
			radio.txStandBy();

			radio.startListening();
		}
	}

	if (trigger_fhss_lost) {
		trigger_fhss_lost = false;
		Serial.print("RC lost, waiting at channel ");
		Serial.println(*fhss_ptr);
		target_pwm_motor = 0;
		target_pwm_servo = SERVO_MIDDLE;
	}

	if (millis() - last_ts > 1000) {
		last_ts = millis();
		Serial.print("recv ");
		Serial.println(rx_count);
		Serial.print("failed count ");
		 for (int i = 0; i < sizeof(fhss_failed_count); ++i) {
		 Serial.print(fhss_failed_count[i]);
		 Serial.print(" ");
		 }
		 Serial.println();
		/*Serial.print(rc_payload.roll);
		 Serial.print(" ");
		 Serial.print(rc_payload.pitch);
		 Serial.print(" ");
		 Serial.print(rc_payload.yaw);
		 Serial.print(" ");
		 Serial.print(rc_payload.thr);
		 Serial.println();*/
		rx_count = 0;
		memset(fhss_failed_count, 0, sizeof(fhss_failed_count));
	}
}
