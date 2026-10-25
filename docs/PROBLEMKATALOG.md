# Gemeinsamer Problemkatalog

Fortlaufend ergänzen, IDs nicht neu vergeben. Benutzerbeobachtung und Diagnose
getrennt halten. P0: vor Hardwarefreigabe klären; P1: zentrale Funktion/Leistung;
P2: Bedienbarkeit/Wartung. Status „reproduziert“ meint Software, keine Hardwaremessung.

| ID | Quelle | Prio | Problem / Beleg | Status / geplante Behandlung |
|---|---|---|---|---|
| U-001 | Benutzer | P1 | Vor MOVEL muss MOVEJ ausgeführt werden | Reproduziert: Nullsetzung erzeugt singuläre Modell-Home-Pose. Referenzwinkel und reale Home-Geometrie klären, zulässigen Direktstart prüfen. |
| U-002 | Benutzer | P1 | MOVEL ist deutlich langsamer als MOVEJ | Beispiel reproduziert: 1,68 vs. 1,02 s; konservative Ableitungsgrenzen identifiziert. Exakte Benutzerfahrt noch ausstehend. |
| A-001 | Analyse | P0 | Home-Schrittwert und geometrischer Gelenkwinkel werden gleichgesetzt; keine q_ref/n_ref-Modellierung | Aktiver Code: Homing.cpp:101, MotorController.cpp:109. Separate mechanische Referenzwinkel und Messprozedur vorsehen. |
| A-002 | Analyse | P1 | Global getrennte Betragsmaxima erhöhen T_fahrt unnötig | Kinematics.cpp:116, Movement.cpp:88/101. Lokale gekoppelte Extrema statt pauschaler Maxima prüfen. |
| A-003 | Analyse | P1 | Einzel-Homing fehlt; alle vier Achsen sind Voraussetzung selbst für SETJ/SETZ | SerialCommands.cpp:43/145; Homing hat nur ein globales homed. Bitmaske und achsabhängige Freigaben planen. |
| A-004 | Analyse | P1 | HOME ALL läuft nacheinander, MotorController kann nur eine Home-Achse halten | Homing ORDER/nextAxis und homeAxis_. Eigenen Zustand je Achse und ausdrücklichen Parallelmodus vorsehen. |
| A-005 | Anforderung | P1 | Keine benannten/nummerierten Positionen, keine Persistenz | SAVE/POSITION/LIST/DELETE und versionierten Positionsspeicher planen. EEPROM-Entscheidung angefragt. |
| A-006 | Analyse | P0 | Möglicher ISR-Fehler/DONE-Race | Movement.cpp:186 und :203. Abschlussgrund atomar übernehmen; Unterbrechungen im Test gezielt injizieren. |
| A-007 | Analyse | P0 | Abschalten aller Treiber garantiert kein Halten der Z-Achse und keinen gebremsten Stillstand | MotorController.cpp:160. Mechanische Haltefähigkeit/Bremse prüfen; STOP-Semantik mit Benutzer abgleichen. |
| A-008 | Analyse | P0 | Werkzeugversatz, Z-Skalierung, Lastgrenzen und mechanische Nullwinkel unbestätigt | RobotConfig.h. Hardwaredatenblatt und Kalibrierprotokoll vor Freigabe. L1/L2 sind bestätigt. |
| A-009 | Analyse | P1 | Keine echte Positionsrückmeldung; Schrittzähler werden als aktuelle Position verwendet | MotorController.position. Status als commanded/estimated kennzeichnen; kein Istwertnachweis. |
| A-010 | Analyse | P1 | 100-us-ISR hat variable Laufzeit, u.a. 64-bit-Rechnung beim Framewechsel | MotorController.cpp:181/247. Budget mit GPIO/Logikanalysator messen, ISR vereinfachen; derzeit nicht als gemessener Overrun behaupten. |
| A-011 | Analyse | P1 | IK, Ausgaben und 80-ms-Queue konkurrieren im Hauptprogramm | Movement.generateFrame/SerialCommands.status. Worst-Case-Rechenzeit und STOP-Latenz messen; begrenzte TX-Queue planen. |
| A-012 | Analyse | P1 | Host-double und Mega-double haben unterschiedliche Präzision | Numerische Grenzfälle auf 32-bit-Rechnung und Zielhardware prüfen. Keine formale Intervallgarantie aus Sicherheitsfaktor allein ableiten. |
| A-013 | Analyse | P1 | 20-ms-Gelenkinterpolation ist nur eine Näherung an die mathematische TCP-Gerade | Verifizierbare Referenztoleranz, angepasste Segmentierung und Pulsprüfung; reale Bahn separat messen. |
| A-014 | Analyse | P1 | Endschalter-Freigabe/Polarität/defekte Leitungen nicht diagnostisch vollständig erfasst | Aktiv LOW, INPUT_PULLUP. Verdrahtungsmodell bestätigen; initial LOW, dauerhaft HIGH/LOW, Bouncing und Fehlerszenarien testen. |
| A-015 | Analyse | P1 | Referenzfahrt nur eine Erkennung, kein langsamer zweiter Antastvorgang | Homing Seeking/BackingOff. Zweistufiges Homing und Wiederholgenauigkeit planen. |
| A-016 | Analyse | P1 | Gerade ist nicht zwischen allen erreichbaren Endpunkten zulässig | Inneres Loch, Singularitäten, Gelenkgrenzen, Hindernisse. Ohne explizite Umfahrplanung PATH_BLOCKED melden, nicht heimlich MOVEJ fahren. |
| A-017 | Analyse | P2 | POINT REACH und IK prüfen nicht alle Gelenk-/Z-/Bahnlimits | SerialCommands.cpp:299. Aussageumfang klar benennen, Diagnosen ergänzen; nicht als Fahrfreigabe nutzen. |
| A-018 | Analyse | P2 | Statusmeldungen unterscheiden angenommen/geplant/ausgeführt nur teilweise; Fehler können doppelt gemeldet werden | SerialCommands.reply/reportEvents. Einheitliche Auftrags-ID und eindeutige Endmeldung vorsehen. |
| A-019 | Archiv | P0 | Alte Homing-while-Schleifen blockieren Serial und haben keine Weg-/Zeitgrenze | Historische INO/alte main.cpp; nicht in die Neufassung übernehmen. Aktive Version bereits nicht blockierend. |
| A-020 | Archiv | P1 | Alte Winkel-Schritt-Faktoren rechnen zuerst ganzzahlig, z.B. 16000/360 -> 44 statt 44,444... | NeuerCodeLinusScaraRobot.ino:75ff. Früh zu float/double konvertieren, Umrechnungsfixtures verwenden. |
| A-021 | Archiv | P0 | Einzel-Homing kann global isHomed setzen; altes Homing schaltet selbst E-STOP frei | Alte main.cpp/homeAxis/handleCommand. In Neufassung achsweise Referenz, explizites RESETSTOP und erneutes Homing. |
| A-022 | Archiv | P1 | Alter deaktivierter Servo liegt auf A0 und damit auf Mega-D54 (J1 STEP) | Paket-INO:32. Bei Wiederaufnahme Greifer Pin-Kollisionen statisch prüfen. Aktive Serial-Firmware hat keinen Servo. |
| A-023 | Archiv | P1 | Python-Bridge serialisiert STOP hinter laufenden Serial-Lesevorgängen und verwirft Eingabepuffer | robot_bridge.py send_command. Adapter entfällt; dieses Muster nicht in Serial-/Auftragslogik übernehmen. |
| A-024 | Archiv | P2 | Mehrere vollständige INO-Versionen im selben Sketch-Ordner erzeugen doppelte setup/loop-Definitionen bei gemeinsamem Arduino-Build | Ein eindeutig aktiver PlatformIO-Einstieg, Archiv außerhalb des Buildpfads. |
| A-025 | Archiv | P1 | Altes Z-Homing setzt Ziel 0, beendet aber bei -20000 und bedient Z danach während anderer Referenzfahrten nicht weiter | NeuerCodeLinusScaraRobot.ino:135ff. Tatsächliche mechanische Home-Pose rekonstruieren, kein stiller Nullsetzungsimport. |

## Weitere Benutzerprobleme

Nächste ID: U-003. Bitte möglichst mit Befehl, Startpose/Homingzustand, Sollverhalten,
Istverhalten und Serial-Ausgabe ergänzen. Hardwaremessung, Beobachtung und
Softwarehypothese bleiben jeweils separat gekennzeichnet.
