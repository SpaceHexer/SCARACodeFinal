# SCARA Robot Controller Serial

Neue Mega-2560-Firmware mit Serial-Bedienung und quintischen Gelenkprofilen.
**Keine kartesische Streckenplanung, keine Stepper-Library, keine Webseite.**
Bei X/Y-Zielen wird die IK einmal für das Ziel gelöst. Danach fahren die Gelenke
koordiniert; die Werkzeugbahn darf gekrümmt sein.

Bestätigt: L1=228 mm, L2=136,5 mm, Werkzeugversatz 0/0 und vorläufige geometrische
Home-Winkel 0°. Pins/Getriebe/Homing-Offsets wurden aus der früheren Firmware
übernommen und müssen am Roboter angepasst werden. Aktuelle Firmware steht in
`include/` und `src/`, historische Analysebelege getrennt in `analysis/` und
`docs/archive/`. Frühere Repositories wurden nicht verändert.

## Start auf dem Computer

1. ZIP entpacken und Ordner mit `platformio.ini` in VS Code / PlatformIO öffnen.
2. `include/RobotConfig.h` lesen und mechanische Konfiguration einstellen.
3. `pio run -e megaatmega2560` baut die Firmware. Uno ist nicht unterstützt.
4. Serial Monitor: **115200 Baud**, Zeilenende LF oder CR/LF.

Es wurde keine Firmware auf Hardware geladen und kein Roboter physisch bewegt.
Siehe [Validierung](docs/VALIDIERUNG.md) für Softwareprüfungen und ihre Grenzen.

## Zentrale Konfiguration

Alle Einstellungen stehen in `include/RobotConfig.h`:

- `AXIS`: STEP/DIR/ENABLE/Endschalter, Impulse pro rad bzw. mm, `qRef`, Grenzen,
  Maximalgeschwindigkeit, Beschleunigung, optionaler Ruck und Homingparameter.
- `HOME_CONFIRMED[4]`: standardmäßig false. Nach Prüfung der jeweiligen Achse auf
  true setzen; nicht unbestätigte Richtungen/Offsets ungeprüft freigeben.
- `PARALLEL_CONFIRMED`: standardmäßig false. Erst nach mechanischer Prüfung
  paralleles Homing freigeben. `PARALLEL_GROUPS` enthält erlaubte gleichzeitige
  Achsmengen; HOME ALL wird abgelehnt, wenn nicht alle vier zusammen erlaubt sind.
  HOME ALL SEQUENTIAL ist der ausdrücklich sequenzielle Modus.
- `GEOMETRY_CONFIRMED`: true gemäß den aktuellen Angaben zu Armlängen/Versatz.
- `STOP_BUTTON_1_ENABLED`, `STOP_BUTTON_2_ENABLED`: **direkt true/false ändern**.
- `STOP_BUTTON_1_PIN=22`, `STOP_BUTTON_2_PIN=23`: aktuell freie Mega-Pins.

Bits in einer Achsmaske: J1=1, J2=2, J3=4, Z=8. Beispiel: 15 sind alle Achsen.
`qRef` ist ein Gelenkwinkel in rad bzw. Z in mm, nicht die Schrittzahl. Nach
Homing ist der Zähler n_ref=0; die Umrechnung lautet **q=qRef+n/K**.

Zusätzlich begrenzt das Programm Fahrten vor dem bekannten Referenz-Endschalter:
Die aus dem Homing-Offset folgende Schalterposition ist qRef-homeOffsetSteps/K.
`limits()` kombiniert diesen Wert mit den Benutzergrenzen und fünf Impulsen
Abstand. Es erweitert keine Grenze. Beispiel: J2-Offset 200 Impulse bei qRef=0
bedeutet, dass der untere Endschalter nur ungefähr 5,6° entfernt ist. Die alte
pauschale Angabe -180° wäre damit nicht konsistent. Winkel, Offset und reale
Endschalterlage gemeinsam kalibrieren.

MaxSpeed ist rad/s bzw. mm/s, MaxAcceleration rad/s² bzw. mm/s², MaxJerk rad/s³
bzw. mm/s³. MaxJerk=0 deaktiviert die optionale Ruckgrenze. Die alten 2000 Impulse/s
und 1000 Impulse/s² sind Ausgangseinstellungen, keine gemessenen Lastgrenzen.
Das interne `Real` ist auch im Hosttest float32, entsprechend dem AVR-Mega.

## STOP-Taster

Je aktivem Taster: Eingang gegen **GND**, mit internem INPUT_PULLUP. Geschlossen
ist LOW. Keine Verbindung zur Motorversorgung. Aktive Pins werden auf Konflikte
mit Motor-/Endschalterpins und Serial0 geprüft. Ein deaktivierter Eingang wird
weder initialisiert noch gelesen, auch nicht bei Start/Reset/Status/ISR.

Beide Eingänge werden zu Beginn jedes Timer-Takts geprüft. Erste LOW-Erkennung
verriegelt STOP ohne Einschalt-Entprellverzögerung. Serial-STOP ruft dieselbe
zentrale Abschaltung auf. Schritte/Queue enden, Treiber werden deaktiviert,
Referenzen ungültig. Loslassen setzt nichts fort. Nach stabiler Freigabe aller
aktiven Taster für 20 ms: RESETSTOP, danach HOME oder bewusste manuelle
Referenzierung mit SETREF. Ein deaktivierter Taster
blockiert das nicht. Serial-STOP bleibt bei jeder Tasterkonfiguration verfügbar.
Boot und STATUS zeigen Aktivierung und gedrückte Eingänge.

**Z-Halten bei Treiberabschaltung mechanisch prüfen.** Während normaler Fahrten
bleiben zuvor freigegebene Achsen bestromt; STOP schaltet gemäß Benutzerauftrag
alle ab. Das ist kein unabhängiger zertifizierter Hardware-Not-Halt. GPIO-/Timer-
Latenzen und Treiber-Pulszeiten müssen am echten Mega gemessen werden.

## Bedienung

Winkel in Grad, Z/X/Y in mm, Dauer in Sekunden. Dezimalpunkt, keine angehängten
Einheiten. Genau ein Befehl pro Zeile. OK bestätigt die Annahme; PLAN zeigt die
geprüfte Gesamtdauer, DONE MOVE bzw. DONE HOME den Abschluss. ERR und STOPPED
sind keine Erfolgsmeldungen. Neue normale Fahrbefehle erst nach Abschluss senden.

| Befehl | Bedeutung |
|---|---|
| HELP / PING / STATUS | Hilfe, Verbindung und referenzierter/geschätzter Zustand |
| HOME ALL | Alle vier Achsen parallel, wenn Konfiguration das ausdrücklich erlaubt |
| HOME ALL SEQUENTIAL | Alle Achsen nacheinander |
| HOME J1 / HOME J2 / HOME J3 / HOME Z | Nur diese Achse referenzieren |
| SETREF ALL 0 0 0 0 | Aktuelle bekannte Stellung als J1/J2/J3=0°, Z=0 mm übernehmen; keine Bewegung |
| SETREF J1 30 / SETREF Z 20 | Nur diese Achse mit ihrer bekannten aktuellen Stellung referenzieren |
| SETJ J1 30 [T] | Einzelne referenzierte Drehachse anfahren, Standardwunschdauer 1 s |
| SETZ 20 [T] | Nur referenzierte Z-Achse bewegen |
| MOVEJ 10 20 -30 40 3 [EXACT] | Vier Zielgelenke; J1/J2/J3 in Grad, Z in mm, T_wunsch=3 s |
| MOVEXY 210 145 | Einmalige Ziel-IK, Z und absolute Orientierung bleiben erhalten |
| MOVEXY 210 145 Z 20 PSI 0 T 3 | Zielpose und Wunschdauer angeben |
| SAVE 0 / POSITION 0 [T] | Aktuelle referenzierte TCP-Pose speichern bzw. über Ziel-IK/Gelenkprofile anfahren |
| LIST / DELETE 0 | Gespeicherte Positionen auflisten bzw. löschen |
| STOP / RESETSTOP | Verriegeln und nach Freigabe zurücksetzen; danach erneut HOME oder SETREF nötig |
| FK j1 j2 j3 z | Vorwärtskinematik, Winkelargumente in Grad |
| IK x y z psi | Passende zulässige Zielgelenke ausgeben, psi in Grad |
| REACH x y z psi | Prüft, ob eine zulässige Ziel-IK existiert; keine Kollisionsprüfung |
| COEFF T q0 v0 a0 q1 v1 a1 | Sechs normierte Koeffizienten einer skalaren Rechnung |
| PROFILE T t [q0 v0 a0 q1 v1 a1] | Skalarprofil auswerten; ohne Zusatzwerte 0→1, Ruhe-zu-Ruhe |

`MOVEL` und `CHECKLINE` werden ausdrücklich als entfernt zurückgewiesen, nicht
unter anderer Bedeutung ausgeführt. `CONFIG CONFIRM` entfällt; die Hardware-
Konfiguration wird zentral im Code geprüft und freigegeben.

Es gibt 16 Slots (0..15) **ausschließlich im RAM**. Nach Stromausfall/Neustart sind
sie leer. STOP/RESETSTOP löschen gespeicherte Zielposen nicht, aber neue Fahrten
benötigen erneut Referenzen. SAVE erfordert Stillstand und vollständige Referenzen.
Keine EEPROM-Schreibvorgänge. Ein gespeicherter Punkt wird bei jedem Abruf neu
über IK und Achsgrenzen geprüft. Er garantiert keine kollisionsfreie Anfahrt.

### Homing überspringen: bekannte Stellung manuell übernehmen

SETREF bewegt den Roboter nicht und sucht keinen Endschalter. Du gibst seine
**tatsächliche aktuelle Stellung** vor. Nur wenn er wirklich bei allen Winkeln
0° und Z=0 mm steht, passt dieses Beispiel:

```text
SETREF ALL 0 0 0 0
STATUS
MOVEJ 1 1 1 0 2
```

Andere Ausgangsstellung, beispielsweise J1=10°, J2=30°, J3=-40°, Z=20 mm:

```text
SETREF ALL 10 30 -40 20
```

Einzelne Achsen: `SETREF J1 10`, `SETREF J2 30`, `SETREF J3 -40`, `SETREF Z 20`.
Nur die jeweils angegebenen Achsen erhalten eine Referenz. Intern werden ihre
Schrittzähler passend zu qRef gesetzt, auf den nächsten Schritt gerundet; qRef
in der Konfiguration bleibt unverändert. STEP und Treiberfreigabe ändern sich
dabei nicht. Das ist eine Benutzerangabe, keine gemessene Encoderposition.

SETREF ist nur im Stillstand erlaubt, auch nicht während BEGIN/SEG, Planung
oder Homing. Ungültige Werte/Softgrenzen, ein aktiver Achsendschalter, aktive
STOP-Taster und bestehende Fehler sperren die Übernahme. Nach STOP/Fehler erst
`RESETSTOP`, dann erneut HOME oder SETREF. Loslassen/Neustart referenziert nicht
automatisch. Die Hardwarekonfiguration und Grenzen müssen weiterhin stimmen.
Unbekannte tatsächliche Stellung erfordert Homing; blindes Nullsetzen kann zu
falschen Bewegungen führen.

### TIMING_ERROR zurücksetzen

TIMING_ERROR verriegelt absichtlich alle Folgeaufträge. Eine längere
Wunschfahrzeit hebt diese Sperre nicht auf. Die Korrektur vom 28.10.2026
verlegt die Frame-Vorbereitung aus der 100-us-ISR in die Hauptschleife. Den
aktualisierten Code kompilieren und auf den Mega hochladen. Eigene Werte in
RobotConfig.h vorher sichern und in den neuen Stand übernehmen. Danach erneut
HOME oder SETREF verwenden. Bei erneutem Zeitfehler nicht die Überwachung
abschalten; STATUS und den genauen Ablauf zur weiteren Diagnose festhalten.

### Beispiel nach Hardwarekalibrierung

Befehle einzeln senden und jeweils DONE abwarten:

```text
HOME ALL SEQUENTIAL
MOVEXY 210 145 Z 20 PSI 0 T 3
SAVE 0
SETJ J1 10 2
SAVE 1
Position 0
Position 1
STATUS
STOP
RESETSTOP
HOME ALL SEQUENTIAL
```

MOVEXY funktioniert vom referenzierten gestreckten Start ohne vorgeschaltetes
MOVEJ: Es wird kein kartesischer Start-Jacobian invertiert. Die tatsächliche
Werkzeugbewegung ist hier ausdrücklich keine Gerade.

## Alle sechs quintischen Randwerte tatsächlich ausführen

Normale Befehle verwenden v_start=a_start=v_ziel=a_ziel=0. Für andere Randwerte
wird vor dem Start eine Folge aus maximal drei Segmenten vorbereitet:

```text
BEGIN
SEG 10 90 -90 20 3 V0 0 0 0 0 A0 0 0 0 0 V1 2 0 0 0 A1 1 0 0 0
SEG 20 90 -90 20 3 V0 2 0 0 0 A0 1 0 0 0 V1 0 0 0 0 A1 0 0 0 0
RUN
```

Die vier Werte hinter V0/V1 sind J1/J2/J3 in Grad/s und Z in mm/s, hinter A0/A1
entsprechend Grad/s² und mm/s². Zielposition und T sind die fünf ersten Werte von
SEG. **Die gezeigten Ziele sind Beispiele, keine geprüfte kollisionsfreie Hardwarebahn.**

- Startposition des ersten Segments stammt aus referenzierten Schrittzählern.
- Erstes Segment beginnt aus Halten: eine erfundene Startgeschwindigkeit wird
  nicht akzeptiert. Bei Anschlusssegmenten stammen v0/a0 aus dem vorigen Profil.
- V0 und A0 entweder beide ausdrücklich angeben (dann auf Übereinstimmung prüfen)
  oder beide weglassen (dann übernehmen).
- V1/A1 sind standardmäßig 0. Eine Folge darf erst RUN ausführen, wenn ihr letztes
  Segment v=a=0 hat. Auch eine beliebig kleine nicht-null Endrate braucht Nachfolge.
- Nach NEED_SUCCESSOR bleibt BEGIN offen: weiteres Segment hinzufügen oder CANCEL.
- CANCEL verwirft nur noch nicht gestartete BEGIN-Daten. Während Fahrt STOP verwenden.
- Die ganze Folge wird vor der ersten Impulsausgabe geprüft; alle vier Achsen
  verwenden innerhalb eines Segments dieselbe T_fahrt.
- Jede Achse hat ein eigenes Gelenkpolynom. Innere Extremstellen werden durch
  Ableitungsnullstellen bestimmt (keine bloße Stichprobenfreigabe).
- Ohne EXACT darf die gemeinsame Dauer verlängert werden. Mit nicht-null Randraten
  werden für jeden Kandidaten alle Koeffizienten neu gefittet und geprüft. Die
  begrenzte Suche ist keine Garantie, eine existierende Lösung immer zu finden.
- EXACT verlangt eine zulässige Wunschdauer als Vielfaches von 20 ms. Maximal
  120 s je Segment. EXACT betrifft die Referenz-/Frame-Dauer; Timerjitter und
  der abschließende LOW-Takt sind keine garantierte physische Deadline.
  Keine unabhängige Achsgeschwindigkeits-Klemmung während Fahrt.

Eine Endgeschwindigkeit oder Endbeschleunigung ungleich null ist damit nicht nur
COEFF-Demonstration, sondern Bestandteil einer geprüften ausgeführten Folge.

## Architektur

| Modul | Aufgabe |
|---|---|
| RobotConfig / Types | Parameter, Einheiten und gemeinsame Datentypen |
| RobotState | Referenzbit je Achse; Betriebs-/Fehlerzustände liegen bei ihren Besitzern |
| Kinematics | FK und einmalige Ziel-IK, beide Zweige und nächste Winkeldarstellung |
| QuinticProfile | Sechs Koeffizienten, Ableitungen und innere Extremstellen |
| Movement | Vorbereiten/Prüfen gemeinsamer Gelenksegmente und Frame-Ausgabe |
| MotorController | Atomare Zustände, q_ref/n_ref-Umrechnung, GPIO und Timer3 |
| Homing | Zustandsautomat je Achse, zwei Antastungen, Parallel-/Einzelmodus |
| PositionStore | 16 RAM-Slots mit Werkzeugposen |
| StopInputs | Ausschließlich aktivierte STOP-Pins überwachen |
| SerialCommands / Console | Striktes Parsing, begrenzte nicht blockierende Ausgabe |
| main | Subsysteme verbinden und zyklisch bedienen |

Timer3: 100 µs, Frame: 200 Takte/20 ms, Queue: vier Frames. STEP bleibt nominell
mindestens einen Takt HIGH und LOW, DIR wird vorher gesetzt. Die Grenzwerte des
Treiber-Datenblatts stehen in RobotConfig; die Firmware prüft ihre Verträglichkeit
mit dem Zeitraster. Impulsabstände werden ganzzahlig quantisiert; die Planung
hält Reserve für Rundung der Frame-Endpunkte ein. Profilgrenzen beziehen sich auf
die kontinuierliche Referenz, reale Mechanik und Schrittverluste sind nicht gemessen.

Die UART-Ausgabe geht in einen festen Ringpuffer; das Hauptprogramm wartet nicht
auf Serial-TX. LIST wird portioniert. Bei übermäßiger Befehlsflut meldet die Ausgabe
TX_OVERFLOW; STATUS erneut abfragen. Ein überlasteter UART-Empfang ist kein sicherer
Not-Halt-Kanal. Die aktivierten STOP-Pins bleiben unabhängig davon im Timer überwacht.

## Bauen und Tests

```bash
pio run -e megaatmega2560
bash tests/host/run.sh
pio run -e megaatmega2560 -e mega_stop00 -e mega_stop01 -e mega_stop10
# Optional unter Linux mit libsimavr-dev: echte Mega-ELF ausführen.
bash tests/avr-timing/run.sh .pio/build/megaatmega2560/firmware.elf
```

Hosttests: g++, Bash, ASan/UBSan (unter Windows z.B. WSL). Sie simulieren GPIO,
Serial und die reale main-Schleife mit float32-Mathematik; alle vier STOP-
Kombinationen werden unabhängig gebaut. Simulationsfreigaben sind beim AVR-Build
per Compilerfehler verboten. Mega-Builds benötigen keine externe C++-Bibliothek.
GitHub Actions führt dieselben Prüfungen aus, sobald das Repository verfügbar ist.

[Prüfergebnisse](docs/VALIDIERUNG.md), [aktueller Plan](docs/PLAN.md),
[Problemkatalog](docs/PROBLEMKATALOG.md), [Mathematik](docs/MATHEMATIK.md).

## GitHub-Status

Gewünschtes Repository: **SpaceHexer/SCARA-Robot-Controller-Serial**, zunächst privat.
Der Erstellungsversuch via GitHub-CLI scheitert in dieser Umgebung an
`https://api.github.com/users/SpaceHexer: Forbidden`. Auch Plugin-Werkzeuge für
GitHub sind hier nicht aufrufbar. Es wurde deshalb noch kein neues Remote erstellt
und nichts hochgeladen. Der lokale Git-Stand und die ZIP enthalten die Ergebnisse.
Sobald Schreibzugriff funktioniert, kann dieser Stand in das gewünschte neue
Repository gepusht werden. Keine Tokens oder Geheimnisse in Projektdateien ablegen.
