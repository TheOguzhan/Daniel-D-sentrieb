# 🇩🇪

# Daniel Düsentrieb Kode von den Schülern des Istanbul Erkek Gymnasiums

## Konstante für SKS1:

| PIN | art der Stecknadel | Aufgabe                                       |
| --- | ------------------ | --------------------------------------------- |
| A0  | Eingang            | IR Sensor Eingangspin                         |
| A4  | Eingang            | Farbe Sensor Sda Eingangspin                  |
| A5  | Eingang            | Farbe Sensor Scl Eingangspin                  |
| D3  | Ausgang            | Links Motor rückwarts Ausgangpin              |
| D4  | Ausgang            | Rechts Motor rückwarts Ausgangpin             |
| D5  | Ausgang            | Links Motor Geschwindigkeit (PWM) Ausgangpin  |
| D6  | Ausgang            | Rechts Motor Geschwindigkeit (PWM) Ausgangpin |
| D7  | Ausgang            | Links Motor vorwärts Ausgangpin               |
| D8  | Ausgang            | Rechts Motor vorwärts Ausgangpin              |
| D9  | Ausgang            | LED R Ausgangpin                              |
| D10 | Ausgang            | LED G Ausgangpin                              |
| D11 | Ausgang            | LED B Ausgangpin                              |

## Algorithmus für SKS1

- ### Der URROB beginnt für seine Aufgabe

```cpp
// Der URROB macht die Aufgaben, die in dieser Funktion stehen. 
void loop() {
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
void linie_folgen() {
 // Der URROB benutzt das RGBsensor, 
 // um der URRBOB zu überprüfen, ob der Roboter 
 // auf die Linie geht und ob es auf 
 // der Linie etwas gibt.
 float r, g, b;
 farbe_sensor.getRGB(&r, &g, &b);
 if (r < FARBE_OBERE_SCHWELLE || g < FARBE_OBERE_SCHWELLE || b < FARBE_OBERE_SCHWELLE) {
  motoren_richten(0);
 } 
 else {
  float infra = analogRead(INFRAROT_SENSOR);
  if (infra < INFRAROT_WEISS_SCHWELLE) {
   motoren_richten(KORRIGIERUNGSRICHTUNG_RECHTS);  
  } 
else {
   motoren_richten(KORRIGIERUNGSRICHTUNG_LINKS);
  }
 }
}
```

- ### Während der URROB die Linie folgt, bestimmt er die Farbe der Plätchen

- ### Dann zeigt er die Farbe davon
