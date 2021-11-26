#include <i2cmaster.h>

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
#define PLATTCHEN_FARBE_FEHLER 110 /* Deviation von den erwarteten Farben, die Plättchen besitzen */
#define INFRAROT_WEISS_SCHWELLE 100
#define KORRIGIERUNGSRICHTUNG_RECHTS 30
#define KORRIGIERUNGSRICHTUNG_LINKS 50
/* Automatisch berechnet, nicht direkt geändern! */

#define FARBE_SPEZIFISCH_STELLE 110
#define FARBE_WEISS_SCHWELLE 60
#define WEISS_LUX_SCHWELLE 500


void farbesensor_lesen(float *r, float *g, float *b, uint16_t *lux=NULL) {
	uint16_t r_raw, g_raw, b_raw, clear_raw, lux_raw;

	i2c_start_wait(0xb4);
	i2c_write(0x00);
	i2c_start_wait(0xb5);
	r_raw = i2c_readAck() << 8;
	r_raw |= i2c_readAck();
	g_raw = i2c_readAck() << 8;
	g_raw |= i2c_readAck();
	b_raw = i2c_readAck() << 8;
	b_raw |= i2c_readAck();
	clear_raw = i2c_readAck() << 8;
	if (!lux) {
		clear_raw |= i2c_readNak();
	} else {
		clear_raw |= i2c_readAck();
		lux_raw = i2c_readAck() << 8;
		lux_raw |= i2c_readNak();
	}
	
    i2c_stop();

	*r = ((float)r_raw / (float)clear_raw) * 255;
	*g = ((float)g_raw / (float)clear_raw) * 255;
	*b = ((float)b_raw / (float)clear_raw) * 255;
	if (lux) {
		*lux = lux_raw;
	}
}

bool farbensensor_weiss() {
	float r, g, b;
	uint16_t lux;
	farbesensor_lesen(&r, &g, &b, &lux);
	return (r > FARBE_WEISS_SCHWELLE && g > FARBE_WEISS_SCHWELLE && b > FARBE_WEISS_SCHWELLE && lux > WEISS_LUX_SCHWELLE);
}

/*
Treibt den Motoren mit Richtung-Eingabe
@param richtung Richtung zwischen -127 (ganz nach links) und 128 (ganz nach rechts) 
*/
void motoren_richten(int richtung) {
	//TODO: Rausoptimisieren und alle Benutzungen inline machen?
	richtung += 127;
	analogWrite(MOTOR_LINKS_PWM, 255 - richtung);
	analogWrite(MOTOR_RECHTS_PWM, richtung);
}

void plattchen_behandeln() {
	float r, g, b; /* Rot, Grün, Blau */
	farbesensor_lesen(&r, &g, &b);
	
	if (g > FARBE_SPEZIFISCH_STELLE) {
		/* Grünes Plättchen, LED soll grün sein */
		digitalWrite(LED_R, HIGH);
		digitalWrite(LED_G, LOW);
		
	} else if (r > FARBE_SPEZIFISCH_STELLE) {
		/* Rotes Plättchen, LED soll rot sein */
		digitalWrite(LED_R, LOW);
		digitalWrite(LED_G, HIGH);
	} else {
		/* Kein Plättchen, LED ausgeschaltet */
		digitalWrite(LED_R, HIGH);
		digitalWrite(LED_G, HIGH);
	}

}

void linie_folgen() {
	float r, g, b;
	farbesensor_lesen(&r, &g, &b);
	if (farbensensor_weiss()) {
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
	} else {
		//Farbensensor sieht nicht weiß

		//Wir sind auf dem Linie, nichts zu tun als vorwärts zu gehen. (?)
		//TODO: Probiert das echt!
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
	pinMode(INFRAROT_SENSOR, INPUT);

	digitalWrite(LED_R, HIGH);
	digitalWrite(LED_G, HIGH);
	digitalWrite(LED_B, HIGH);
	
	//Motoren orientieren, bei SKS1 nur nach vorne
	digitalWrite(MOTOR_LINKS_RUCKWARTS, 0);
	digitalWrite(MOTOR_LINKS_VORWARTS, 1);
	digitalWrite(MOTOR_RECHTS_RUCKWARTS, 0);
	digitalWrite(MOTOR_RECHTS_VORWARTS, 1);
	
	i2c_init();
    delay(1); 
	
}

void loop() {
	// put your main code here, to run repeatedly:
	linie_folgen();
	plattchen_behandeln();
}
