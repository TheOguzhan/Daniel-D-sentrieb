#include <Wire.h>
#include <Adafruit_TCS34725.h>
#define INFRAROT_SENSOR 0
#define MOTOR_LINKS_RUCKWARTS 3   
#define MOTOR_RECHTS_RUCKWARTS 4   
#define MOTOR_LINKS_PWM 5   
#define MOTOR_RECHTS_PWM 6   
#define MOTOR_LINKS_VORWARTS 7   
#define MOTOR_RECHTS_VORWARTS 8   
#define LED_R 9   
#define LED_G 10   
#define LED_B 11
/*
Treibt den Motoren mit Richtung-Eingabe
@param richtung Richtung zwischen -127 (ganz nach links) und 128 (ganz nach rechts) 
*/

Adafruit_TCS34725 farbe_sensor;

void motoren_richten(int richtung) {
	richtung += 127;
	analogWrite(MOTOR_LINKS_PWM, 255 - richtung);
	analogWrite(MOTOR_RECHTS_PWM, richtung);
}

typedef enum MotorOrientierung : int {
	STOPP = 0,
	VORNE,
	RUCKWARTS
};

void motoren_orientieren(int links, int rechts) {
	switch (links) {
		case MotorOrientierung::STOPP:
			digitalWrite(MOTOR_LINKS_RUCKWARTS, 0);
			digitalWrite(MOTOR_LINKS_VORWARTS, 0);
		break;
		case MotorOrientierung::VORNE:
			digitalWrite(MOTOR_LINKS_RUCKWARTS, 0);
			digitalWrite(MOTOR_LINKS_VORWARTS, 1);
		break;
		case MotorOrientierung::RUCKWARTS:
			digitalWrite(MOTOR_LINKS_RUCKWARTS, 1);
			digitalWrite(MOTOR_LINKS_VORWARTS, 0);
		break;
	}
	switch (rechts) {
		case MotorOrientierung::STOPP:
			digitalWrite(MOTOR_RECHTS_RUCKWARTS, 0);
			digitalWrite(MOTOR_RECHTS_VORWARTS, 0);
		break;
		case MotorOrientierung::VORNE:
			digitalWrite(MOTOR_RECHTS_RUCKWARTS, 0);
			digitalWrite(MOTOR_RECHTS_VORWARTS, 1);
		break;
		case MotorOrientierung::RUCKWARTS:
			digitalWrite(MOTOR_RECHTS_RUCKWARTS, 1);
			digitalWrite(MOTOR_RECHTS_VORWARTS, 0);
		break;
	}
}

void plattchen_behandeln() {

}

void linie_folgen() {

}

void setup() {
	// put your setup code here, to run once:
	pinMode(MOTOR_LINKS_RUCKWARTS, OUTPUT);
	pinMode(MOTOR_RECHTS_RUCKWARTS, OUTPUT);
	pinMode(MOTOR_LINKS_PWM, OUTPUT);
	pinMode(MOTOR_RECHTS_PWM, OUTPUT);
	pinMode(MOTOR_LINKS_VORWARTS, OUTPUT);
	pinMode(MOTOR_RECHTS_VORWARTS, OUTPUT);
	pinMode(LED_R, OUTPUT);
	pinMode(LED_G, OUTPUT);
	pinMode(LED_B, OUTPUT);
	pinMode(INFRAROT_SENSOR, INPUT);

	farbe_sensor.begin();
	farbe_sensor.setInterrupt(false);
	
}

void loop() {
	// put your main code here, to run repeatedly:
	
}
