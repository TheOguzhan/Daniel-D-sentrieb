# Daniel Düsentrieb Kode von den Schülern des Istanbul Erkek Gymnasiums

## Stecknadel Anordnung für SKS2

 | PIN | Art der Stecknadel | Aufgabe                                       |
 | --- | ------------------ | --------------------------------------------- |
 | A0  | Eingang            | IR Sensor Eingangspin                         |
 | A4  | Eingang            | Farbe Sensor Sda Eingangspin                  |
 | A5  | Eingang            | Farbe Sensor Scl Eingangspin                  |
 | D3  | Ausgang            | Links Motor vorwärts Ausgangpin               |
 | D4  | Ausgang            | Links Motor rückwarts Ausgangpin              |
 | D5  | Ausgang            | Links Motor Geschwindigkeit (PWM) Ausgangpin  |
 | D6  | Ausgang            | Rechts Motor Geschwindigkeit (PWM) Ausgangpin |
 | D7  | Ausgang            | Rechts Motor vorwärts Ausgangpin              |
 | D8  | Ausgang            | Rechts Motor rückwärts Ausgangpin             |
 | D9  | Ausgang            | LED G Ausgangpin                              |
 | D10 | Ausgang            | LED B Ausgangpin                              |
 | D11 | Ausgang            | LED R Ausgangpin                              |
 | D12 | Ausgang            | Elektromagnet Ausgangpin                      |

## Abhängigkeit

- [ic2master](https://github.com/landis/arduino/tree/master/libraries/I2cMaster)

## Algorithmus für SKS2

- ### Der URROB beginnt für seine Aufgabe

```cpp


// Der URROB macht die Aufgaben, die in dieser Funktion stehen. 

void loop()
{
 // Die Funktion für die Liniefolgen
 linie_folgen();
 // Die Funktion für die Plätchenhändlung
 plattchen_behandeln();
}
```

- ### Der URROB folgt die Linie

```cpp

// Diese Funktion ist für die Linie Folgen.
// In dieser Funktion handhat der URROB 
// Linie Folgen. 

void linie_folgen()
{
 int jetzt = millis();
 int zeit_differenz;
 if (wieder_beginnen)
 {
  // Das Roboter wurde früher gestoppt, 
  // das Plättchenbehandlungalgorithmus ist jetzt vom Liniefolgerzustand verantwortlich!
  zeit_differenz = ZEIT_PRO_PERIODE;
  wieder_beginnen = false;
 }
 else
 {
  zeit_differenz = jetzt - letzte_folge_zeit;
 }
 letzte_folge_zeit = jetzt;

 if (analogRead(INFRAROT_SENSOR) > 512)
 {
  if (linie_folge_speicher >= -0.00)
   linie_folge_speicher = -0.07;
 }
 else
 {
  if (linie_folge_speicher <= 0.00)
   linie_folge_speicher = 0.07;
 }
 // Normierte Multiplikation durch die Zeitdifferenz,
 // damit ein verlangsamtes Roboter sich nicht ganz anders bewegt.
 linie_folge_speicher *= pow(1.17, ((float)zeit_differenz) / ZEIT_PRO_PERIODE);

 if (linie_folge_speicher > 1.0)
  linie_folge_speicher = 1.0;
 if (linie_folge_speicher < -1.0)
  linie_folge_speicher = -1.0;
 // Motoren richtig treiben
 if (linie_folge_speicher <= 0.0)
 {
  motoren_treiben(128 - abs(linie_folge_speicher * 100), 128);
 }
 else
 {
  motoren_treiben(128, 128 - abs(linie_folge_speicher * 100));
 }
}

```

- ### Während der URROB die Linie folgt, bestimmt er die Farbe der Plätchen

- ### Dann nimmt der URROB Roboter rote Plättchen ein und entfernt er die rote Plättchen

```cpp
void plattchen_behandeln()
{
 float r, g, b; /* Rot, Grün, Blau */
 farbesensor_lesen(&r, &g, &b);

 if (r > FARBE_SPEZIFISCH_STELLE)
 {
  /* Rotes Plättchen, Plättchen einnehmen und entfernen */
  digitalWrite(LED_R, LOW);

  digitalWrite(ELEKTROMAGNET, HIGH);
  motoren_treiben(-140, -140);
  // warten, bis das Plättchen sicher eingenommen wird
  delay(ELEKTROMAGNET_FANGEN_ZEIT_MS);
  // nach rechts um den rechten Rad biegen
  motoren_treiben(140, 0);
  delay(BIEGEN_ZEIT_MS);

  // nach vorne gehen, um das Plättchen weiter zu entfernen
  motoren_treiben(128, 128);
  delay(VORNE_ZEIT_MS);

  // hier stoppen und das Plättchen weglassen
  motoren_treiben(0, 0);
  digitalWrite(ELEKTROMAGNET, LOW);
  delay(ELEKTROMAGNET_ZEIT_MS);

  // wieder zürück kommen
  motoren_treiben(-128, -128);
  delay(VORNE_ZEIT_MS);

  // wieder nach links um das rechten Rad genau so viel biegen, wie das Roboter früher nach rechts gebogen hat
  // damit kommen wir zürück auf dem Linie
  // Das Roboter geht nach links ein bisschen schneller und muss deswegen dazu kompensiert werden
  motoren_treiben(-130, 0);
  // motoren schon nach rechts orientiert, aber dieses Mal ist das linke Motor rückwärts
  // da das linke Motor hier auch mit der Höchstgeschwindigkeit, aber nur rückwärts geht, ist kein Änderung der Motortreibung mehr benötigt.
  delay(BIEGEN_ZEIT_MS);

  // Wieder zur Linie-Folge
  motoren_treiben(128, 128);
  digitalWrite(LED_R, HIGH);
  wieder_beginnen = true;
  linie_folge_speicher = -0.1;
 }
 else if (g > FARBE_SPEZIFISCH_STELLE)
 {
  /* Grünes Plättchen, LED soll grün sein */
  digitalWrite(LED_G, LOW);
  motoren_treiben(-150, -150);
  delay(100);
  motoren_treiben(0, 0);
  delay(FARBE_ZEIGEN_ZEIT);
  digitalWrite(LED_G, HIGH);
  wieder_beginnen = true;
  linie_folge_speicher = 0.1;
 }
}
```

- ### Wichtige Funktionen

- #### Motor treiben

```cpp
/*
Treibt den Motoren mit PWM-Angaben
@param links PWM-Wert der linken Motor. Wenn negativ, wird das linke Motor umgekehrt. Werte zwischen -255 und 255.
@param rechts PWM-Wert der rechten Motor. Wenn negativ, wird das rechte Motor umgekehrt. Werte zwischen -255 und 255.
*/
void motoren_treiben(int links, int rechts)
{
 // Linke Motoren treiben
 if (links < 0)
 {
  digitalWrite(MOTOR_LINKS_RUCKWARTS, 1);
  digitalWrite(MOTOR_LINKS_VORWARTS, 0);
  analogWrite(MOTOR_LINKS_PWM, -links);
 }
 else if (links == 0)
 {
  digitalWrite(MOTOR_LINKS_RUCKWARTS, 0);
  digitalWrite(MOTOR_LINKS_VORWARTS, 0);
  analogWrite(MOTOR_LINKS_PWM, 0);
 }
 else
 {
  // links > 0
  digitalWrite(MOTOR_LINKS_RUCKWARTS, 0);
  digitalWrite(MOTOR_LINKS_VORWARTS, 1);
  analogWrite(MOTOR_LINKS_PWM, links);
 }

 // Rechte Motoren treiben
 if (rechts < 0)
 {
  digitalWrite(MOTOR_RECHTS_RUCKWARTS, 1);
  digitalWrite(MOTOR_RECHTS_VORWARTS, 0);
  analogWrite(MOTOR_RECHTS_PWM, -rechts);
 }
 else if (rechts == 0)
 {
  digitalWrite(MOTOR_RECHTS_RUCKWARTS, 0);
  digitalWrite(MOTOR_RECHTS_VORWARTS, 0);
  analogWrite(MOTOR_RECHTS_PWM, 0);
 }
 else
 {
  // rechts > 0
  digitalWrite(MOTOR_RECHTS_RUCKWARTS, 0);
  digitalWrite(MOTOR_RECHTS_VORWARTS, 1);
  analogWrite(MOTOR_RECHTS_PWM, rechts);
 }
}

```

- #### Sensorwert lesen

```cpp

void farbesensor_lesen(float *r, float *g, float *b, uint16_t *lux = NULL)
{
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
 if (!lux)
 {
  clear_raw |= i2c_readNak();
 }
 else
 {
  clear_raw |= i2c_readAck();
  lux_raw = i2c_readAck() << 8;
  lux_raw |= i2c_readNak();
 }

 //Für SKS2 funktioniert diese Treiber besser

 float roh_dividend = (r_raw + g_raw + b_raw);
 i2c_stop();
 if (clear_raw)
 {
  *r = ((float)r_raw / (float)roh_dividend) * 255;
  *g = ((float)g_raw / (float)roh_dividend) * 255;
  *b = ((float)b_raw / (float)roh_dividend) * 255;
 }
 else
 {
  *r = 0;
  *g = 0;
  *b = 0;
 }

 if (lux)
 {
  *lux = lux_raw;
 }
}


```

- #### Konstanten und Variablen

```cpp

#define INFRAROT_SENSOR A0
#define MOTOR_LINKS_RUCKWARTS 4
#define MOTOR_RECHTS_RUCKWARTS 8
#define MOTOR_LINKS_PWM 5
#define MOTOR_RECHTS_PWM 6
#define MOTOR_LINKS_VORWARTS 3
#define MOTOR_RECHTS_VORWARTS 7
#define LED_R 9
#define LED_B 10
#define LED_G 11
#define ELEKTROMAGNET 12

#define PLATTCHEN_FARBE_FEHLER 110 /* Deviation von den erwarteten Farben, die Plättchen besitzen */
#define INFRAROT_WEISS_SCHWELLE 400
/* Automatisch berechnet, nicht direkt geändern! */

#define FARBE_SPEZIFISCH_STELLE 110
#define FARBE_WEISS_SCHWELLE 60
#define WEISS_LUX_SCHWELLE 500

#define MOTOREN_STOPP_ZEIT 50
#define MOTOREN_STOPP_PWM -200
#define FARBE_ZEIGEN_ZEIT 600

/* Zeit in ms, die das Roboter benötigt, um ein Plättchen zufriedigend weit wegzuschoben */
#define BIEGEN_ZEIT_MS 1100
/* Zeit in ms, die das Elektromagnet benötigt, vollständig an- und auszuschalten*/
#define ELEKTROMAGNET_ZEIT_MS 750
#define ELEKTROMAGNET_FANGEN_ZEIT_MS 500
#define VORNE_ZEIT_MS 300
// Zeit in ms pro Umlauf des Liniefolgers unter Normalbedingungen, nach unten für die Normierung benutzt
#define ZEIT_PRO_PERIODE 12.0
// Wert zwischen -1 und 1, die vom Liniefolger gespeichert wird, damit wir erinnern, wie weit wir vom Linie entfernt sind
float linie_folge_speicher = 0;
// Millisekunden vom Programmanfang seit der letzte Umlauf des Liniefolgers
int letzte_folge_zeit = 0;
// Ist es direkt nach einem Plattchenbehandlung, indem das Roboter vollstanden gestoppt und wieder gestartet wurde?
bool wieder_beginnen = 0;

```

- #### Abhängige Bibliotheken

```cpp
/* Der URROB ist abhängig von dieser Bibliothek. */
#include <i2cmaster.h>
// Der Link der Bibliothek ist oben.
```

- #### ```void setup()``` Funktion des Kodes

```cpp
void setup()
{
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
 pinMode(ELEKTROMAGNET, OUTPUT);

 digitalWrite(LED_R, HIGH);
 digitalWrite(LED_G, HIGH);
 digitalWrite(LED_B, HIGH);

 // Motoren orientieren, bei SKS1 nur nach vorne
 motoren_treiben(128, 128);

 i2c_init();
 delay(1);
}
```

- #### ```void loop()``` Funktion des Kodes

```cpp

void loop()
{
 linie_folgen();
 plattchen_behandeln();
}

```

## Ordnerstruktur

```folder
├── .vscode
│   └── arduino.json
├── .gitignore
├── Daniel-D-sentrieb.ino
├── Dependencies.txt
└── README.md
```

###  ├── .vscode

> Konfiguration Dateien des IDE VSCode

### &nbsp;└── arduino.json

> Konfiguration Dateien des Arduino Plug-In des VSCode

### ├── .gitignore

> Die Dateien, die Git ignorieren wird

### ├── Daniel-D-sentrieb.ino

> Der Kode des URROBs

### ├── Dependencies.txt

> Die Liste der Abhängigkeiten

### └── README .md

> Die Datei, die Sie jetzt Lesen, für den schriftlichen Nachweis

## Komplett Kode

> Wenn Sie möchten, können Sie auch den Kode in Daniel-D-sentrieb.ino gucken.

```cpp
#include <i2cmaster.h>

#define INFRAROT_SENSOR A0
#define MOTOR_LINKS_RUCKWARTS 4
#define MOTOR_RECHTS_RUCKWARTS 8
#define MOTOR_LINKS_PWM 5
#define MOTOR_RECHTS_PWM 6
#define MOTOR_LINKS_VORWARTS 3
#define MOTOR_RECHTS_VORWARTS 7
#define LED_R 9
#define LED_B 10
#define LED_G 11
#define ELEKTROMAGNET 12

#define PLATTCHEN_FARBE_FEHLER 110 /* Deviation von den erwarteten Farben, die Plättchen besitzen */
#define INFRAROT_WEISS_SCHWELLE 400
/* Automatisch berechnet, nicht direkt geändern! */

#define FARBE_SPEZIFISCH_STELLE 110
#define FARBE_WEISS_SCHWELLE 60
#define WEISS_LUX_SCHWELLE 500

#define MOTOREN_STOPP_ZEIT 50
#define MOTOREN_STOPP_PWM -200
#define FARBE_ZEIGEN_ZEIT 600

/* Zeit in ms, die das Roboter benötigt, um ein Plättchen zufriedigend weit wegzuschoben */
#define BIEGEN_ZEIT_MS 1100
/* Zeit in ms, die das Elektromagnet benötigt, vollständig an- und auszuschalten*/
#define ELEKTROMAGNET_ZEIT_MS 750
#define ELEKTROMAGNET_FANGEN_ZEIT_MS 500
#define VORNE_ZEIT_MS 300
//Zeit in ms pro Umlauf des Liniefolgers unter Normalbedingungen, nach unten für die Normierung benutzt
#define ZEIT_PRO_PERIODE 12.0
//Wert zwischen -1 und 1, die vom Liniefolger gespeichert wird, damit wir erinnern, wie weit wir vom Linie entfernt sind
float linie_folge_speicher = 0;
//Millisekunden vom Programmanfang seit der letzte Umlauf des Liniefolgers
int letzte_folge_zeit = 0;
//Ist es direkt nach einem Plattchenbehandlung, indem das Roboter vollstanden gestoppt und wieder gestartet wurde?
bool wieder_beginnen = 0;

void farbesensor_lesen(float *r, float *g, float *b, uint16_t *lux = NULL)
{
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
 if (!lux)
 {
  clear_raw |= i2c_readNak();
 }
 else
 {
  clear_raw |= i2c_readAck();
  lux_raw = i2c_readAck() << 8;
  lux_raw |= i2c_readNak();
 }

 //Für SKS2 funktioniert diese Treiber besser

 float roh_dividend = (r_raw + g_raw + b_raw);
 i2c_stop();
 if (clear_raw)
 {
  *r = ((float)r_raw / (float)roh_dividend) * 255;
  *g = ((float)g_raw / (float)roh_dividend) * 255;
  *b = ((float)b_raw / (float)roh_dividend) * 255;
 }
 else
 {
  *r = 0;
  *g = 0;
  *b = 0;
 }

 if (lux)
 {
  *lux = lux_raw;
 }
}

/*
Treibt den Motoren mit PWM-Angaben
@param links PWM-Wert der linken Motor. Wenn negativ, wird das linke Motor umgekehrt. Werte zwischen -255 und 255.
@param rechts PWM-Wert der rechten Motor. Wenn negativ, wird das rechte Motor umgekehrt. Werte zwischen -255 und 255.
*/
void motoren_treiben(int links, int rechts)
{
 // Linke Motoren treiben
 if (links < 0)
 {
  digitalWrite(MOTOR_LINKS_RUCKWARTS, 1);
  digitalWrite(MOTOR_LINKS_VORWARTS, 0);
  analogWrite(MOTOR_LINKS_PWM, -links);
 }
 else if (links == 0)
 {
  digitalWrite(MOTOR_LINKS_RUCKWARTS, 0);
  digitalWrite(MOTOR_LINKS_VORWARTS, 0);
  analogWrite(MOTOR_LINKS_PWM, 0);
 }
 else
 {
  // links > 0
  digitalWrite(MOTOR_LINKS_RUCKWARTS, 0);
  digitalWrite(MOTOR_LINKS_VORWARTS, 1);
  analogWrite(MOTOR_LINKS_PWM, links);
 }

 // Rechte Motoren treiben
 if (rechts < 0)
 {
  digitalWrite(MOTOR_RECHTS_RUCKWARTS, 1);
  digitalWrite(MOTOR_RECHTS_VORWARTS, 0);
  analogWrite(MOTOR_RECHTS_PWM, -rechts);
 }
 else if (rechts == 0)
 {
  digitalWrite(MOTOR_RECHTS_RUCKWARTS, 0);
  digitalWrite(MOTOR_RECHTS_VORWARTS, 0);
  analogWrite(MOTOR_RECHTS_PWM, 0);
 }
 else
 {
  // rechts > 0
  digitalWrite(MOTOR_RECHTS_RUCKWARTS, 0);
  digitalWrite(MOTOR_RECHTS_VORWARTS, 1);
  analogWrite(MOTOR_RECHTS_PWM, rechts);
 }
}

void plattchen_behandeln()
{
 float r, g, b; /* Rot, Grün, Blau */
 farbesensor_lesen(&r, &g, &b);

 if (r > FARBE_SPEZIFISCH_STELLE)
 {
  /* Rotes Plättchen, Plättchen einnehmen und entfernen */
  digitalWrite(LED_R, LOW);

  digitalWrite(ELEKTROMAGNET, HIGH);
  motoren_treiben(-140, -140);
  //warten, bis das Plättchen sicher eingenommen wird
  delay(ELEKTROMAGNET_FANGEN_ZEIT_MS);
  //nach rechts um den rechten Rad biegen
  motoren_treiben(140, 0);
  delay(BIEGEN_ZEIT_MS);

  //nach vorne gehen, um das Plättchen weiter zu entfernen
  motoren_treiben(128, 128);
  delay(VORNE_ZEIT_MS);

  //hier stoppen und das Plättchen weglassen
  motoren_treiben(0, 0);
  digitalWrite(ELEKTROMAGNET, LOW);
  delay(ELEKTROMAGNET_ZEIT_MS);

  //wieder zürück kommen
  motoren_treiben(-128, -128);
  delay(VORNE_ZEIT_MS);

  //wieder nach links um das rechten Rad genau so viel biegen, wie das Roboter früher nach rechts gebogen hat
  //damit kommen wir zürück auf dem Linie
  //Das Roboter geht nach links ein bisschen schneller und muss deswegen dazu kompensiert werden
  motoren_treiben(-130, 0);
  //motoren schon nach rechts orientiert, aber dieses Mal ist das linke Motor rückwärts
  //da das linke Motor hier auch mit der Höchstgeschwindigkeit, aber nur rückwärts geht, ist kein Änderung der Motortreibung mehr benötigt.
  delay(BIEGEN_ZEIT_MS);

  //Wieder zur Linie-Folge
  motoren_treiben(128, 128);
  digitalWrite(LED_R, HIGH);
  wieder_beginnen = true;
  linie_folge_speicher = -0.1;
 }
 else if (g > FARBE_SPEZIFISCH_STELLE)
 {
  /* Grünes Plättchen, LED soll grün sein */
  digitalWrite(LED_G, LOW);
  motoren_treiben(-150, -150);
  delay(100);
  motoren_treiben(0, 0);
  delay(FARBE_ZEIGEN_ZEIT);
  digitalWrite(LED_G, HIGH);
  wieder_beginnen = true;
  linie_folge_speicher = 0.1;
 }
}

void linie_folgen()
{
 int jetzt = millis();
 int zeit_differenz;
 if (wieder_beginnen)
 {
  // Das Roboter wurde früher gestoppt, das Plättchenbehandlungalgorithmus ist jetzt vom Liniefolgerzustand verantwortlich!
  zeit_differenz = ZEIT_PRO_PERIODE;
  wieder_beginnen = false;
 }
 else
 {
  zeit_differenz = jetzt - letzte_folge_zeit;
 }
 letzte_folge_zeit = jetzt;

 if (analogRead(INFRAROT_SENSOR) > 512)
 {
  if (linie_folge_speicher >= -0.00)
   linie_folge_speicher = -0.07;
 }
 else
 {
  if (linie_folge_speicher <= 0.00)
   linie_folge_speicher = 0.07;
 }
 // Normierte Multiplikation durch die Zeitdifferenz, damit ein verlangsamtes Roboter sich nicht ganz anders bewegt.
 linie_folge_speicher *= pow(1.17, ((float)zeit_differenz) / ZEIT_PRO_PERIODE);

 if (linie_folge_speicher > 1.0)
  linie_folge_speicher = 1.0;
 if (linie_folge_speicher < -1.0)
  linie_folge_speicher = -1.0;
 // Motoren richtig treiben
 if (linie_folge_speicher <= 0.0)
 {
  motoren_treiben(128 - abs(linie_folge_speicher * 100), 128);
 }
 else
 {
  motoren_treiben(128, 128 - abs(linie_folge_speicher * 100));
 }
}

void setup()
{
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
 pinMode(ELEKTROMAGNET, OUTPUT);

 digitalWrite(LED_R, HIGH);
 digitalWrite(LED_G, HIGH);
 digitalWrite(LED_B, HIGH);

 // Motoren orientieren, bei SKS1 nur nach vorne
 motoren_treiben(128, 128);

 i2c_init();
 delay(1);
}

void loop()
{

 linie_folgen();
 plattchen_behandeln();
}
```

## Hergestellt von [IELRobotics](https://github.com/IEL-Robotics), [Trashcognito](https://github.com/trashcognito), [ohasanov-hbrw](https://github.com/ohasanov-hbrw), [TheOguzhan](https://github.com/TheOguzhan)
