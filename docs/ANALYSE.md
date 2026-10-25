# Bestandsanalyse vom 25.10.2026

## Umfang und unveränderte Basis

Primäre Basis ist der wiederhergestellte Serial-Stand vom 24.10.2026 unter
`/workspace/SCARA-Roboter-Serial/SCARA-Roboter_PlattformIO`. Alle dortigen eigenen
C++-Header und Implementierungen wurden statisch untersucht; Hosttests und
Mega-Build wurden ausgeführt. Firmware wurde nicht verändert oder hochgeladen.

Das ursprüngliche Repository `/workspace/SCARA-Roboter` (HEAD
6373d1ab4d331bd4fc7e015f7df6da88d3a9229c) wurde nur gelesen. Historische Arduino-
Varianten wurden auf Struktur, Homing, Umrechnung und Protokoll verglichen.
Der frühere Python-Adapter wurde ergänzend auf Befehls-/Antwortfluss untersucht;
Weboberfläche und Drittanbieter-Pythonpakete sind nicht Gegenstand einer neuen
vollständigen Web-Sicherheitsprüfung, da sie aus dem Zielprogramm entfallen.
Lokale Änderungen im Windows-Projekt TestingArduinoForScara sind hier nicht
verfügbar. Der Manifest dokumentiert die tatsächlich vorliegenden Quellstände.

## Ausgeführte Prüfungen

- `bash tests/host/run.sh`: 7 Subsystem-Testgruppen sowie main-/Serial-Integration
  bestanden; ASan/UBSan aktiv, g++ mit -Wall -Wextra -Werror.
- `pio run -e megaatmega2560 -j 4`: bestanden; RAM 1420/8192 Bytes,
  Flash 32176/253952 Bytes; keine externen Stepper-Bibliotheken.
- Zusatzprobe `analysis/probe.cpp`: reproduziert die beiden Benutzerprobleme.
- Keine Messung von Timerlatenz, Pulsbreiten, Schritten unter Last oder tatsächlicher
  TCP-Position auf einem realen Mega/Roboter.

## Benutzerproblem 1: MOVEJ vor MOVEL

Beleg: Homing.cpp:101 setzt jedes Gelenk auf Schrittwert 0. MotorController.cpp:109
rechnet n/K ohne geometrischen Referenzwinkel. Deshalb ist q1=q2=q3=0 die angenommene
Home-Pose. Im Modell bedeutet q2=0 einen gestreckten Arm mit sin(q2)=0.
Movement.cpp:61 prüft sofort die IK am Start, Kinematics.cpp:48 lehnt diesen
Singularitätsbereich ab. Die Probe liefert `HOME-logical-zero MOVEL: SINGULARITY`.

Das ist kein Bedarf, den Zielpunkt erst per MOVEJ bekannt zu machen. Entweder ist
die Zuordnung zwischen realer Home-Pose und Modellwinkeln falsch/unvollständig,
oder die wirkliche Home-Pose ist tatsächlich singulär. Diese Fälle sind zu trennen.
Eine Sperre einfach zu entfernen wäre keine Lösung: auf einer radial nach innen
führenden Gerade gilt am gestreckten Rand lokal theta2 ~ sqrt(s). Beim normalen
Ruhe-zu-Ruhe-Quintik ist s ~ t_lauf³; daraus kann theta2 ~ t_lauf^(3/2) und eine
unbeschränkte Gelenkbeschleunigung am Start entstehen. Ein solcher Sonderfall
braucht eine eigene gültige Zeitplanung oder eine explizit vereinbarte Home-/Park-
Strategie. Eine versteckte MOVEJ-Ausweichfahrt würde die behauptete Gerade ändern.

## Benutzerproblem 2: langsames MOVEL

Identischer Start in beiden Versuchen: q=(0,pi/2,-pi/2,30), TCP=(228,136.5,30),
psi=0. Identisches Ziel: TCP=(210,145,30), psi=0. MOVEJ verwendet die IK dieses
Ziels, also dieselbe Zielpose. T_wunsch=0,1 s; Konfiguration unverändert.

| Versuch | Berechnete Dauer |
|---|---:|
| MOVEJ | 1,020 s |
| MOVEL | 1,680 s |
| Dichte numerische Abschätzung des MOVEL-Profils | ca. 1,042 s |

Die numerische Abschätzung nutzt 20.001 normierte Profilpunkte mit zentralen
Differenzen. Sie ist Diagnosehilfe und KEIN Beweis für kontinuierliche Grenz-
einhaltung. Sie zeigt, dass in diesem Beispiel ein erheblicher Teil des
Verlangsamens aus Überabschätzung stammen kann, nicht aus gemessener MCU-Last.
Die tatsächlichen Benutzerbefehle/Zeiten stehen noch aus; die Zahlen behaupten
nicht, die konkrete Hardwarefahrt nachgestellt zu haben.

Kinematics.cpp:116-134 summiert Betragsgrenzen und verliert dadurch Kompensationen.
Movement.cpp:88-90 übernimmt je Achse globale Maxima. Movement.cpp:101-111 kombiniert
sie erneut mit globalen Fortschrittsmaxima, obwohl diese nicht notwendigerweise
am selben Ort/zu derselben Zeit auftreten. Das ist konservativ, aber oft langsam.
In MOVEJ ist q_i(s) linear: q_i''=q_i'''=0. Außerdem fährt dessen TCP im Allgemeinen
keine Gerade. Identische Endpunkte bedeuten daher nicht identische Bewegung.

Ziel der Neufassung: tatsächliche zeitabhängige Ableitungen auswerten und deren
Extrema adaptiv mit überprüfbaren Restgrenzen eingrenzen; danach weiterhin Grenzen
und T_fahrt validieren. Keine bloße Erhöhung der Geschwindigkeitslimits und kein
Ersatz von kontinuierlicher Prüfung durch eine unbelegte grobe Stichprobe.

## Laufzeit und Hardwaregrenzen

Der 100-us-Interrupt liest micros, Endschalter und volatile Zähler. Bei Framewechseln
kommen Queue-Kopien, 64-bit-Differenzen und DIR-Ausgaben hinzu; Homing rechnet Rampen
im selben Interrupt. Aufwand ist sichtbar, eine Budgetüberschreitung aber nicht
auf Hardware nachgewiesen. Auf dem Mega bedeutet double normalerweise nur 32-bit-
Präzision; Hosttests mit 64-bit-double decken deren Rundungsverhalten nicht ab.

Die vier 20-ms-Frames sind maximal 80 ms vorausberechneter Puffer. IK und Serial-
Antworten laufen im Hauptprogramm und können das Nachfüllen verzögern. Serial STOP
ist von Empfang, Zeilenende, Hauptschleife und ggf. blockierender Ausgabe abhängig;
er ersetzt keinen unabhängigen Hardware-Not-Halt.

## Race zwischen Fehler und DONE

Movement.cpp:186 liest motors.error vor der weiteren Arbeit. Danach kann die ISR
einen Underrun/Endstop setzen und running=false machen. Movement.cpp:203 kann
anschließend Done setzen, ohne den Fehler noch einmal gemeinsam mit dem Endzustand
zu prüfen. Dieses Interleaving ist aus dem Code möglich, im bisherigen Hosttest
aber nicht durch echte Interrupt-Präemption reproduziert. Eine atomare Terminal-
meldung oder ein Ereignis mit Abschlussgrund muss in der Neufassung den Ausschlag
geben; nicht allein !running.

## Referenzlink

https://howtomechatronics.com/projects/scara-robot-how-to-build-your-own-arduino-based-robot/

Abruf in dieser Umgebung: Proxy-CONNECT 403. Die aktuelle Seite, ihre Zeichnungen,
Dimensionen, Getriebe und mögliche Achskopplungen sind deshalb noch nicht verifiziert.
Der Link bleibt eine ausdrücklich vorgesehene Designreferenz; Hardwarewerte werden
nicht aus vermutetem Seiteninhalt übernommen.
