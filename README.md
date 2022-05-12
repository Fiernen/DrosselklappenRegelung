# DrosselklappenRegelung
Regelung der Winkelposition:
1 Redundante Potenziometer als Winkelsensor an der Motorwelle
2 Ansteuerung des Antriebs über PWM 
3 zeitdiskreter P-Lageregler mit einer maximal zulässigen Regelabweichung von +/-
0,1° 
4 zeitdiskreter PI-Drehzahlregler
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
