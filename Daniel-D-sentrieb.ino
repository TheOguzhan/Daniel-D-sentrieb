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

//TODO: Diese Werte im echten Leben probieren!
#define PLATTCHEN_FARBE_FEHLER 30 /* Deviation von den erwarteten Farben, die Plättchen besitzen */
#define INFRAROT_WEISS_SCHWELLE 100
#define KORRIGIERUNGSRICHTUNG_RECHTS 30
#define KORRIGIERUNGSRICHTUNG_LINKS 50
/* Automatisch berechnet, nicht direkt geändern! */

#define FARBE_OBERE_SCHWELLE (255-PLATTCHEN_FARBE_FEHLER)
#define FARBE_UNTERE_SCHWELLE (PLATTCHEN_FARBE_FEHLER)

/*
Treibt den Motoren mit Richtung-Eingabe
@param richtung Richtung zwischen -127 (ganz nach links) und 128 (ganz nach rechts) 
*/

Adafruit_TCS34725 farbe_sensor;

void motoren_richten(int richtung) {
	//TODO: Rausoptimisieren und alle Benutzungen inline machen?
	richtung += 127;
	analogWrite(MOTOR_LINKS_PWM, 255 - richtung);
	analogWrite(MOTOR_RECHTS_PWM, richtung);
}

void plattchen_behandeln() {
	float r, g, b; /* Rot, Grün, Blau */
	farbe_sensor.getRGB(&r, &g, &b);
	
	if (r < FARBE_UNTERE_SCHWELLE && g > FARBE_OBERE_SCHWELLE && b < FARBE_UNTERE_SCHWELLE) {
		/* Grünes Plättchen, LED soll grün sein */
		digitalWrite(LED_R, LOW);
		digitalWrite(LED_G, HIGH);
		
	} else if (r > FARBE_OBERE_SCHWELLE && g < FARBE_UNTERE_SCHWELLE && b < FARBE_UNTERE_SCHWELLE) {
		/* Rotes Plättchen, LED soll rot sein */
		digitalWrite(LED_R, HIGH);
		digitalWrite(LED_G, LOW);
	} else {
		/* Kein Plättchen, LED ausgeschaltet */
		digitalWrite(LED_R, LOW);
		digitalWrite(LED_G, LOW);
	}

}

void linie_folgen() {
	float r, g, b;
	farbe_sensor.getRGB(&r, &g, &b);
	if (r < FARBE_OBERE_SCHWELLE || g < FARBE_OBERE_SCHWELLE || b < FARBE_OBERE_SCHWELLE) {
		//Farbensensor sieht nicht weiß

		//Wir sind auf dem Linie, nichts zu tun als vorwärts zu gehen. (?)
		//TODO: Probiert das echt!
		motoren_richten(0);
	} else {
		//Farbensensor sieht weiß
		//Infrarot-Sensor muss probiert werden
		//TODO: Resettieren des Infrarots benötigt?
		float infra = analogRead(INFRAROT_SENSOR);
		if (infra < INFRAROT_WEISS_SCHWELLE) {
			//Infrarot sieht auch weiß
			//Da das Infrarot am linken seite liegt, würde es das Linie zuerst "sehen" und wieder schwarz sehen, wenn das Deviation zur rechten Seite wäre
			//Das Deviation ist deshalb zur linken Seite und das Roboter soll sich zur rechten Seite biegen
			//TODO: Prüfe, ob das geht!
			motoren_richten(KORRIGIERUNGSRICHTUNG_RECHTS);
			
		} else {
			//Infrarot sieht nicht weiß - Linie auf der linken Seite gefunden
			//Das Roboter soll sich nach links bewegen
			motoren_richten(KORRIGIERUNGSRICHTUNG_LINKS);
		}
	}
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

	digitalWrite(LED_B, LOW);

	//Motoren orientieren, bei SKS1 nur nach vorne
	digitalWrite(MOTOR_LINKS_RUCKWARTS, 0);
	digitalWrite(MOTOR_LINKS_VORWARTS, 1);
	digitalWrite(MOTOR_RECHTS_RUCKWARTS, 0);
	digitalWrite(MOTOR_RECHTS_VORWARTS, 1);
	
	farbe_sensor.begin();
	farbe_sensor.setInterrupt(false);
	
}

void loop() {
	// put your main code here, to run repeatedly:
	linie_folgen();
	plattchen_behandeln();
}
