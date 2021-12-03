#include <i2cmaster.h>

#define INFRAROT_SENSOR A0
#define MOTOR_LINKS_RUCKWARTS 4   
#define MOTOR_RECHTS_RUCKWARTS 8   
#define MOTOR_LINKS_PWM 5   
#define MOTOR_RECHTS_PWM 6   
#define MOTOR_LINKS_VORWARTS 3  
#define MOTOR_RECHTS_VORWARTS 7   
#define LED_R 11   
#define LED_B 10   
#define LED_G 9
#define ELEKTROMAGNET 12

//TODO: Diese Werte im echten Leben probieren!
#define PLATTCHEN_FARBE_FEHLER 110 /* Deviation von den erwarteten Farben, die Plättchen besitzen */
#define INFRAROT_WEISS_SCHWELLE 400
/* Automatisch berechnet, nicht direkt geändern! */

#define FARBE_SPEZIFISCH_STELLE 110
#define FARBE_WEISS_SCHWELLE 60
#define WEISS_LUX_SCHWELLE 500

#define MOTOREN_STOPP_ZEIT 50
#define MOTOREN_STOPP_PWM -200

/* Zeit in ms, die das Roboter benötigt, um ein Plättchen zufriedigend weit wegzuschoben */
#define BIEGEN_ZEIT_MS 1250 
/* Zeit in ms, die das Elektromagnet benötigt, vollständig an- und auszuschalten*/
#define ELEKTROMAGNET_ZEIT_MS 750

//Zeit in ms pro Umlauf des Liniefolgers unter Normalbedingungen, nach unten für die Normierung benutzt
#define ZEIT_PRO_PERIODE 12.0
//Wert zwischen -1 und 1, die vom Liniefolger gespeichert wird, damit wir erinnern, wie weit wir vom Linie entfernt sind
float linie_folge_speicher = 0;
//Millisekunden vom Programmanfang seit der letzte Umlauf des Liniefolgers
int letzte_folge_zeit = 0;
//Ist es direkt nach einem Plattchenbehandlung, indem das Roboter vollstanden gestoppt und wieder gestartet wurde?
bool wieder_beginnen = 0;

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
	if (clear_raw) {
		*r = ((float)r_raw / (float)clear_raw) * 255;
		*g = ((float)g_raw / (float)clear_raw) * 255;
		*b = ((float)b_raw / (float)clear_raw) * 255;
	} else {
		*r = 0;
		*g = 0;
		*b = 0;
	}
	
	if (lux) {
		*lux = lux_raw;
	}
}

/*
Treibt den Motoren mit PWM-Angaben
@param links PWM-Wert der linken Motor. Wenn negativ, wird das linke Motor umgekehrt. Werte zwischen -255 und 255.
@param rechts PWM-Wert der rechten Motor. Wenn negativ, wird das rechte Motor umgekehrt. Werte zwischen -255 und 255.
*/
void motoren_treiben(int links, int rechts) {
	// Linke Motoren treiben
	if (links < 0) {
		digitalWrite(MOTOR_LINKS_RUCKWARTS, 1);
		digitalWrite(MOTOR_LINKS_VORWARTS, 0);
		analogWrite(MOTOR_LINKS_PWM, -links);
	} else if (links == 0) {
		digitalWrite(MOTOR_LINKS_RUCKWARTS, 0);
		digitalWrite(MOTOR_LINKS_VORWARTS, 0);
		analogWrite(MOTOR_LINKS_PWM, 0);
	} else {
		// links > 0
		digitalWrite(MOTOR_LINKS_RUCKWARTS, 0);
		digitalWrite(MOTOR_LINKS_VORWARTS, 1);
		analogWrite(MOTOR_LINKS_PWM, links);
	}

	// Rechte Motoren treiben
	if (rechts < 0) {
		digitalWrite(MOTOR_RECHTS_RUCKWARTS, 1);
		digitalWrite(MOTOR_RECHTS_VORWARTS, 0);
		analogWrite(MOTOR_RECHTS_PWM, -rechts);
	} else if (rechts == 0) {
		digitalWrite(MOTOR_RECHTS_RUCKWARTS, 0);
		digitalWrite(MOTOR_RECHTS_VORWARTS, 0);
		analogWrite(MOTOR_RECHTS_PWM, 0);
	} else {
		// rechts > 0
		digitalWrite(MOTOR_RECHTS_RUCKWARTS, 0);
		digitalWrite(MOTOR_RECHTS_VORWARTS, 1);
		analogWrite(MOTOR_RECHTS_PWM, rechts);
	}
}

void plattchen_behandeln() {
	float r, g, b; /* Rot, Grün, Blau */
	farbesensor_lesen(&r, &g, &b);

	if (r > FARBE_SPEZIFISCH_STELLE) {
		/* Rotes Plättchen, Plättchen einnehmen und entfernen */
		digitalWrite(ELEKTROMAGNET, HIGH);
		motoren_treiben(0,0);
		//warten, bis das Plättchen sicher eingenommen wird
		//TODO: Probiert das Mechanismus!
		delay(ELEKTROMAGNET_ZEIT_MS);

		//nach rechts um den rechten Rad biegen
		motoren_treiben(255, 0);
		delay(BIEGEN_ZEIT_MS);

		//hier stoppen und das Plättchen weglassen
		motoren_treiben(0, 0);
		digitalWrite(ELEKTROMAGNET, LOW);
		delay(ELEKTROMAGNET_ZEIT_MS);

		//wieder nach links um das rechten Rad genau so viel biegen, wie das Roboter früher nach rechts gebogen hat
		//damit kommen wir zürück auf dem Linie
		//TODO: probieren, ob das geht!
		motoren_treiben(0, 255);
		//motoren schon nach rechts orientiert, aber dieses Mal ist das linke Motor rückwärts
		//da das linke Motor hier auch mit der Höchstgeschwindigkeit, aber nur rückwärts geht, ist kein Änderung der Motortreibung mehr benötigt.
		delay(BIEGEN_ZEIT_MS);

		//Wieder zur Linie-Folge
		motoren_treiben(255, 255);

		wieder_beginnen = true;
	}
}

void linie_folgen() {
	int jetzt = millis();
	int zeit_differenz;
	if (wieder_beginnen) {
		//Das Roboter wurde früher gestoppt, das Plättchenbehandlungalgorithmus ist jetzt vom Liniefolgerzustand verantwortlich!
		zeit_differenz = ZEIT_PRO_PERIODE;
		wieder_beginnen = false;
	} else {
		zeit_differenz = jetzt - letzte_folge_zeit;
	}
	letzte_folge_zeit = jetzt;
	
	if (analogRead(INFRAROT_SENSOR) > 512) {
    	if (linie_folge_speicher >= -0.00) linie_folge_speicher = -0.07;
    	//ans -= 0.05;
  	} else {
    	if (linie_folge_speicher <= 0.00) linie_folge_speicher = 0.07;
    	//ans += 0.05;
  	}
	//Normierte Multiplikation durch die Zeitdifferenz, damit ein verlangsamtes Roboter sich nicht ganz anders bewegt. 
	linie_folge_speicher *= pow(1.17, ((float)zeit_differenz) / ZEIT_PRO_PERIODE);

	if (linie_folge_speicher > 1.0) linie_folge_speicher = 1.0;
	if (linie_folge_speicher < -1.0) linie_folge_speicher = -1.0;
	//Motoren richtig treiben
	if (linie_folge_speicher <= 0.0) {
		motoren_treiben(128 - abs(linie_folge_speicher * 100), 128);
	} else {
		motoren_treiben(128, 128 - abs(linie_folge_speicher * 100));
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
	motoren_treiben(128, 128);

	i2c_init();
    delay(1); 
		
}

void loop() {
	// put your main code here, to run repeatedly:
	linie_folgen();
	plattchen_behandeln();
}