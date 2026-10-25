# Noch offene Angaben

Vor Implementierung bzw. der betroffenen Hardwarefreigabe zu klären:

1. Name, Besitzerkonto und öffentlich/privat für das neue GitHub-Repository.
2. Sollen Positionen dauerhaft im EEPROM bleiben? Vorschlag: ja; Homing nach Neustart
   trotzdem erforderlich. Slotzahl nach Budget, vorläufiger Vorschlag 32.
3. Welche geometrischen Gelenkwinkel hat die Mechanik nach dem heutigen Homing?
   Ist der Arm tatsächlich gestreckt oder wird nur softwareseitig q2=0 behauptet?
4. Werkzeugversatz delta_x/delta_y und Verwendung eines Öffnungsservos. Die bestätigten
   Armlängen sind L1=228 mm, L2=136,5 mm, nicht automatisch alle Tutorialmaße.
5. Arduino Mega weiterhin vorhanden? Treibertypen, Microsteps, Getriebe,
   Spindelsteigung, DIR-Vorzeichen, Homing-Richtungen und Schalterverdrahtung.
6. Können J1/J2/J3/Z mechanisch gleichzeitig homen? Braucht z.B. Z zuerst Freiraum?
   Ist die Z-Achse bei deaktivierten Treibern selbsthemmend oder gebremst?
7. Tatsächliche Gelenkbereiche, belastbare Geschwindigkeits-/Beschleunigungsgrenzen,
   gewünschte TCP-Toleranz und ggf. Encoderrückmeldung.
8. Soll eine nicht zulässige Gerade abgelehnt werden, oder gehören automatische
   Umfahrungen zum gewünschten „kürzesten Weg“? Dafür braucht es Hindernisdaten.
9. Für U-002: konkrete MOVEJ-/MOVEL-Befehle, Startpose, T_wunsch, Ausgabe von PLAN
   und beobachtete Fahrzeiten. Gleiche Endpunkte sind noch nicht dieselbe Bahn.

Weitere Benutzerprobleme werden unter U-003 ff. ergänzt, bevor die Neufassung startet.
