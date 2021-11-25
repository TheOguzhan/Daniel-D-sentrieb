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
#define ELEKTROMAGNET 12

/* Zeit in ms, die das Roboter benötigt, um ein Plättchen zufriedigend weit wegzuschoben */
#define BIEGEN_ZEIT_MS 1250 
/* Zeit in ms, die das Elektromagnet benötigt, vollständig an- und auszuschalten*/
#define ELEKTROMAGNET_ZEIT_MS 750
/* Deviation von den erwarteten Farben, die Plättchen besitzen */
#define PLATTCHEN_FARBE_FEHLER 30 

#define INFRAROT_WEISS_SCHWELLE 800
#define KORRIGIERUNGSRICHTUNG_RECHTS 30
#define KORRIGIERUNGSRICHTUNG_LINKS 50

/* Automatisch berechnet, nicht direkt geändern! */
#define FARBE_OBERE_SCHWELLE (255-PLATTCHEN_FARBE_FEHLER)
#define FARBE_UNTERE_SCHWELLE (PLATTCHEN_FARBE_FEHLER)

Adafruit_TCS34725 farbe_sensor;

/*!
@brief Treibt den Motoren mit Richtung-Eingabe
@param richtung Richtung zwischen -127 (ganz nach links) und 128 (ganz nach rechts) 
*/
void motoren_richten(int richtung) {
  richtung += 127;
  analogWrite(MOTOR_LINKS_PWM, 255 - richtung); 
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


void linie_folgen() {
	float r, g, b;
	motoren_orientieren(VORNE, VORNE);
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
		if (infra > INFRAROT_WEISS_SCHWELLE) {
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

void plattchen_behandeln() {
	float r, g, b; /* Rot, Grün, Blau */
	farbe_sensor.getRGB(&r, &g, &b);

	if (r > FARBE_OBERE_SCHWELLE && g < FARBE_UNTERE_SCHWELLE && b < FARBE_UNTERE_SCHWELLE) {
		/* Rotes Plättchen, Plättchen einnehmen und entfernen */
		digitalWrite(ELEKTROMAGNET, HIGH);
		//motoren schon nach vorne orientiert; direkt nach vorne gehen, damit wir das Plättchen einfangen können
		motoren_richten(0);
		//warten, bis das Plättchen sicher eingenommen wird
		//TODO: Probiert das Mechanismus!
		delay(ELEKTROMAGNET_ZEIT_MS);

		//nach rechts um den rechten Rad biegen
		motoren_orientieren(VORNE, STOPP);
		motoren_richten(128);
		delay(BIEGEN_ZEIT_MS);

		//hier stoppen und das Plättchen weglassen
		motoren_orientieren(STOPP, STOPP);
		digitalWrite(ELEKTROMAGNET, LOW);
		delay(ELEKTROMAGNET_ZEIT_MS);

		//wieder nach links um das rechten Rad genau so viel biegen, wie das Roboter früher nach rechts gebogen hat
		//damit kommen wir zürück auf dem Linie
		//TODO: probieren, ob das geht!
		motoren_orientieren(RUCKWARTS, STOPP);
		//motoren schon nach rechts orientiert, aber dieses Mal ist das linke Motor rückwärts
		//da das linke Motor hier auch mit der Höchstgeschwindigkeit, aber nur rückwärts geht, ist kein Änderung der Motortreibung mehr benötigt.
		delay(BIEGEN_ZEIT_MS);

		//Wieder zur Linie-Folge
		motoren_orientieren(VORNE, VORNE);
		//Neutral eingesetzt, damit das Liniefolge-Treiber so einfach wie möglich die Kontrolle zürücknehmen kann
		motoren_richten(0);
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
  pinMode(ELEKTROMAGNET, OUTPUT);
}

void loop() {
  	// put your main code here, to run repeatedly:
	linie_folgen();
	plattchen_behandeln();
}
