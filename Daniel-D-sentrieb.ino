#include <i2cmaster.h>

#define INFRAROT_SENSOR A1
#define MOTOR_LINKS_RUCKWARTS 3   
#define MOTOR_RECHTS_RUCKWARTS 7   
#define MOTOR_LINKS_PWM 5   
#define MOTOR_RECHTS_PWM 6   
#define MOTOR_LINKS_VORWARTS 4  
#define MOTOR_RECHTS_VORWARTS 8   
#define LED_R 11   
#define LED_B 10   
#define LED_G 9
//TODO: Diese Werte im echten Leben probieren!
#define PLATTCHEN_FARBE_FEHLER 110 /* Deviation von den erwarteten Farben, die Plättchen besitzen */
#define INFRAROT_WEISS_SCHWELLE 400
/* Automatisch berechnet, nicht direkt geändern! */

#define FARBE_SPEZIFISCH_STELLE 110
#define FARBE_WEISS_SCHWELLE 60
#define WEISS_LUX_SCHWELLE 500

#define FARBE_ZEIGEN_ZEIT 600
#define MOTOREN_STOPP_ZEIT 50
#define MOTOREN_STOPP_PWM -200
#define FARBE_ZEIGEN_REFRAKTARZEIT 100

float linie_folge_speicher = 0;

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
	
	#ifdef DEBUG
	Serial.print("Farbensensor R: ");
	Serial.print(r_raw);
	Serial.print(" G: ");
	Serial.print(g_raw);
	Serial.print(" B: ");
	Serial.print(b_raw);
	Serial.print(" Lux: ");
	Serial.print(lux_raw);
	Serial.print(" Clear: ");
	Serial.println(clear_raw);
	Serial.print("Normiert R: ");
	Serial.print(*r);
	Serial.print(" G: ");
	Serial.print(*g);
	Serial.print(" B: ");
	Serial.println(*b);
	#endif
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
int time_letzte = 0;
void plattchen_behandeln() {
	float r, g, b; /* Rot, Grün, Blau */
	farbesensor_lesen(&r, &g, &b);
	/*
	if (millis() - time_letzte < FARBE_ZEIGEN_REFRAKTARZEIT) {
		//Es ist zu kurz nach dem letzten Anschalten des LEDs gewesen! 
		digitalWrite(LED_R, HIGH);
		digitalWrite(LED_G, HIGH);
		return;
	}
	*/
	bool aktiv = false;
	if (g > FARBE_SPEZIFISCH_STELLE) {
		/* Grünes Plättchen, LED soll grün sein */
		aktiv = true;
		digitalWrite(LED_R, HIGH);
		digitalWrite(LED_G, LOW);
		
		
	} else if (r > FARBE_SPEZIFISCH_STELLE) {
		/* Rotes Plättchen, LED soll rot sein */
		aktiv = true;
		digitalWrite(LED_R, LOW);
		digitalWrite(LED_G, HIGH);
	} else {
		/* Kein Plättchen, LED ausgeschaltet */
		digitalWrite(LED_R, HIGH);
		digitalWrite(LED_G, HIGH);
	}

	if (aktiv) {
		motoren_treiben(-150, -150);
    	delay(100);
		motoren_treiben(0, 0);
		delay(FARBE_ZEIGEN_ZEIT);
		linie_folge_speicher = 0.1;
		time_letzte = millis();
		digitalWrite(LED_R, HIGH);
		digitalWrite(LED_G, HIGH);
	}
}



void linie_folgen() {
	float ans = linie_folge_speicher;
	Serial.println(analogRead(INFRAROT_SENSOR));
	if (analogRead(INFRAROT_SENSOR) > 512) {
		Serial.println("Schwarz");
    	if (ans >= -0.00) ans = -0.07;
    	//ans -= 0.05;
  	} else {
		  Serial.println("Weiss");
    	if (ans <= 0.00) ans = 0.07;
    	//ans += 0.05;
  	}
	ans *= 1.17;
	if (ans > 1.0) ans = 1.0;
	if (ans < -1.0) ans = -1.0;
	linie_folge_speicher = ans;
	Serial.print("ans");
	Serial.println(ans);
	if (ans <= 0.0) {
		motoren_treiben(128 - abs(ans * 100), 128);
	} else {
		motoren_treiben(128, 128 - abs(ans * 100));
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

	#ifdef DEBUG
	Serial.begin(9600);
	#endif
	
}

void loop() {
	// put your main code here, to run repeatedly:
	linie_folgen();
	plattchen_behandeln();
}
