# DrosselklappenRegelung


Regelung der Winkelposition:

1 Redundante Potenziometer als Winkelsensor an der Motorwelle

2 Ansteuerung des Antriebs über PWM 

3 zeitdiskreter P-Lageregler mit einer maximal zulässigen Regelabweichung von +/-0,1° 

4 zeitdiskreter PI-Drehzahlregler

ghr




Bedienung und Visualisierung:

5 Sollwertvorgabe der Drosselklappenstellung über Potentiometer mit einer 
Auflösung von 0,1°

6 Anzeige des Soll- und Istwertes der Drosselklappenstellung auf dem LCD-Display mit 
einer Auflösung von 0,1°

7 Grafische Visualisierung der Soll- und Istwerte, der Stellgröße und der 
Regelabweichung über die RS232-Schnittstelle

8 Parametrierung der Regler über RS232-Schnittstelle

9 Inbetriebnahmemodus mit periodischer sprungförmiger Sollwertvorgabe und einer 
Frequenz von 1Hz

10 Dauerhafte Speicherung der Reglerparameter

11 Drahtbrucherkennung und –alarmierung für die Potenziometer



Bei der Implementierung der Softwarefunktionen sollen floating point Operationen 
vermieden werden. Die Umsetzung der Softwarefunktionen soll nach dem V-Modell 
erfolgen.


H-Brücke :
Erklärung zur Funktionsweise:
An den IN Stecker muss ein PWM-Signal angelegt werden. Ist dabei ein Verhältnis von 50%
5V zu 50% 0V eingestellt, wird der Motor bestromt und bringt dabei ein Haltemoment auf.
Wird das Verhältnis größer als 50% eingestellt, beginnt der Motor sich zu drehen. Je größer
das Verhältnis wird, desto schneller dreht er sich.
Bei einem Verhältnis kleiner 50% dreht der Motor in die andere Richtung. (siehe Bild)
Als Startsignal kann
EN
verwendet werden. Erst wenn dieses Signal auf GND gelegt wird,
startet der Motor Die PWM sollte eine minimale Frequenz von 200 Hz haben.
Stecker 6 Strommessung 0,1V / 1A (über Sense Anschluss 10)
Stecker 10 IN
Stecker 12
EN
Stecker 13 5V
Stecker 14 GND
Stecker 17 Poti 2 Abgriff
Stecker 18 Poti
Stecker 19 Poti
Stecker 20 Poti 1 Abgriff
Imax 4 A
Uein 24V
Betriebsmodus L6203: Two Phase Chopping
}Versorgung mit 5V und GND