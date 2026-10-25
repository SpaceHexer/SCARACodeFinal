# Plan zur Neufassung – vor Beginn der Programmierung

Status: Vorschlag zur Besprechung, keine Freigabe zur Implementierung behauptet.
Weitere Benutzerprobleme werden zuerst in PROBLEMKATALOG.md aufgenommen.

## Ziel und feste Rahmenbedingungen

- Arduino Mega 2560 als zunächst beibehaltene Plattform; Bezug zur verlinkten
  HowToMechatronics-Konstruktion nach erfolgreichem Seitenzugriff verifizieren.
- Keine Stepper-Library, keine Webseite und kein Serial-Adapter.
- Kommentierter, verständlicher C++-Code in kleinen Subsystemen; keine unnötigen
  Framework-Abhängigkeiten. Rechenkern unabhängig von Arduino testbar.
- Intern rad/mm/s, Serial-Winkel in Grad; Variablen T_wunsch, T_probe, T_fahrt und
  t_lauf bleiben erhalten. L1=228 mm, L2=136,5 mm bestätigt.
- Ruhe-zu-Ruhe-Bewegungen und allgemeine quintische Koeffizienten. Allgemeine
  nicht-null Endraten bleiben reine Rechenfunktion, solange nicht separat gewünscht.
- Vorläufig bisher vereinbarter sofortiger STOP, danach erneutes Homing; Halten
  der Z-Mechanik und Hardware-Not-Halt müssen dazu konkret geklärt werden.

## Was „kürzester kartesischer Weg“ bedeutet

Zwischen zwei freien TCP-Punkten ist die Gerade die kürzeste räumliche Verbindung.
POSITION n soll dieselbe geprüfte Geradenplanung wie MOVEL verwenden, einschließlich
Z und gewählter Werkzeugorientierung. Eine lineare Gelenkinterpolation ist dafür
nicht ausreichend. Ein erreichbarer Zielpunkt garantiert keine zulässige Gerade:
inneres Loch, Singularitäten, Achsgrenzen oder Hindernisse können die Verbindung
verhindern. Dann eindeutiger Fehler und keine Bewegung.

Eine automatische kürzeste Umfahrung verlangt ein definiertes Hindernis-/Arbeits-
raummodell und ist eine eigene Entscheidung. Sie wird nicht stillschweigend durch
einen MOVEJ-Ersatz oder eine Z-Ausweichbewegung angenähert. Bitte vor Implementierung
klären, ob zuerst „Gerade oder Ablehnung“ genügt oder Umfahrungen dazugehören sollen.

## Phase 0 – Anforderungen und neues Repository festlegen

1. Neue Benutzerprobleme mit stabilen U-IDs sammeln und bisherigen Befunden zuordnen.
2. Repositoryname/Sichtbarkeit und GitHub-Zugriff klären. Erst nach tatsächlicher
   Erstellung eine URL nennen. Analysedokumente bilden den ersten Stand, nicht
   gleich eine neue Firmware. Bestehende Repositories bleiben erhalten.
3. Hardwareblatt vervollständigen: Home-Winkel, Winkelkonventionen, Werkzeugversatz,
   reale Übersetzungen, Z-Kalibrierung, Schalter, Achskopplung, Treiberzeiten.
4. Plan mit Benutzer besprechen, erst danach neue Programmdateien schreiben.

Ergebnis: akzeptierter Funktionsumfang und nachvollziehbarer Problemkatalog.

## Phase 1 – Zustände, Kalibrierung und Homing neu entwerfen

- RobotConfig beschreibt Mechanik/Pinbelegung und separat Referenzzähler n_ref und
  echte Gelenkwinkel q_ref; die Konfiguration darf keine globale Kreisabhängigkeit
  zu Types erzeugen.
- RobotState kennt je Achse Referenzstatus, STOP/Fehler und laufenden Auftrag.
  Ein einzelnes homed=true reicht nicht. Nach Neustart/Schrittverlust/Abschaltung
  Referenzstatus angemessen invalidieren; EEPROM enthält keine vermeintlich
  weiterhin gültige aktuelle Istposition.
- Homing erhält je Achse eine nicht blockierende Zustandsmaschine: Ausgangsschalter
  prüfen, ggf. freifahren, suchen, langsam erneut antasten, Referenz zuweisen,
  freifahren und abschließen. Zeiten und Wege überwachen; STOP jederzeit bedienen.
- Einzel-Homing und paralleles Homing unterstützen. HOME ALL soll den gewünschten
  gemeinsamen Modus anbieten; HOME J1/J2/J3/Z einzelne Achsen. Sequenzieller Modus
  bleibt explizit anwählbar. Parallele Kombinationen dürfen erst nach mechanischer
  Prüfung freigegeben werden; keine pauschale Zusage kollisionsfreier Gleichzeitigkeit.
- Nach Einzel-Homing darf eine referenzierte Einzelachse bewegt werden. MOVEL und
  Positionsfahrten benötigen den für FK/IK vollständigen Referenzzustand.

U-001 behandeln: Zuerst reale Home-Winkel richtig abbilden. Ist die reale Home-Pose
regulär, muss MOVEL ohne vorherigen manuellen MOVEJ funktionieren. Ist sie wirklich
vollständig gestreckt, ist eine gesonderte Entscheidung nötig: dokumentierte
Home-/Parkstrategie oder mathematisch zulässige Singularitätsausfahrt. Ein
gewöhnliches quintisches Fortschrittsprofil ist dort nicht für jede Richtung
beschleunigungsbegrenzt. Keine heimliche Ausweichfahrt unter dem Namen MOVEL.

## Phase 2 – Kinematik und schnelle, überprüfbare Planung

- FK/IK mit festem kontinuierlichem Zweig, echten Grenzen, Werkzeugversatz und
  ggf. tatsächlich vorhandener Motor-/Gelenkkopplung implementieren und gegen
  bekannte Posen sowie unabhängige Jacobi-/Differenzenrechnungen testen.
- CartesianPath stellt Gerade und kontinuierliche Orientierung bereit.
- QuinticProfile berechnet sechs Koeffizienten und Ableitungen. Fortschritt s und
  Zeitanteil u sind getrennt; stationäre und reine Z-/Orientierungsfahrten behandeln.
- TrajectoryPlanner prüft die gesamte Bahn. Lokale analytische Ableitungen und
  adaptive Extremstellen-/Intervallprüfung sollen die unnötige Summierung
  unabhängiger globaler Worst-Case-Werte ersetzen. Dichte Stichproben dienen
  zum Gegenprüfen, nicht als unbegründete kontinuierliche Garantie.
- Per-Achs-Limits und optionale Rucklimits bleiben bindend; die Dauer ist für alle
  Achsen gleich. Ohne explizite Wunschdauer eine grenzbasierte Standardfahrt,
  mit T_wunsch gemeinsames Strecken, mit EXACT Ablehnung statt heimlicher Änderung.
- Diagnose zeigt limitierende Achse, Geschwindigkeits-/Beschleunigungsanteil,
  berechnete Dauer, Planungszeit und ggf. Singularitätsabstand. Damit wird U-002
  messbar statt durch bloßes Erhöhen der Grenzwerte kaschiert.

## Phase 3 – Motor-Backend und Ausführung

- MotorController rechnet q_ref+(n-n_ref)/K und erzeugt STEP/DIR direkt. Timer und
  Pins werden exklusiv verwaltet, Richtungszeichen und Treiberzeiten sind konfiguriert.
- MotionExecutor wertet eine gemeinsame Zeitbasis aus und puffert kleine vorbereitete
  Segmente. Teure IK und Langzeitplanung bleiben außerhalb der ISR.
- ISR-Arbeit und kritische Abschnitte reduzieren und ein Zeitbudget nachweisen.
  Unterschiedliche Raten der vier Achsen sind erlaubt; deren gemeinsamer Zeitplan
  wird nicht durch unabhängiges Clampen einzelner Achsen verändert.
- Abschlussgrund atomar übergeben: fertig, STOP, Endschalter, Unterlauf, Timingfehler.
  Eine ISR-Abschaltung darf niemals als DONE MOVE bestätigt werden.
- Referenzfehler gegen konfigurierte Toleranz prüfen; elektrische Pulszeiten und
  tatsächliche TCP-Bahn separat messen. Bei unzureichendem Mega-Budget erst
  Profiling/Optimierung; Boardwechsel nur nach Absprache.

## Phase 4 – Positionen und einfache Serial-Bedienung

- SerialCommands behält HOME, MOVEJ, MOVEL, SETJ, SETZ, STATUS, STOP, RESETSTOP und
  Rechenbefehle bei. Menschenlesbare, eindeutige Antworten und abschließende
  Erfolg-/Fehlermeldung pro Auftrag; begrenzte Ein-/Ausgabepuffer.
- PositionStore speichert TCP x/y/z/psi und bei Bedarf einen bevorzugten Zweig/
  Referenzdatensatz. Ein gespeicherter Punkt wird für jede neue Anfahrt erneut
  gegen aktuelle Konfiguration, Startpose und ganze Bahn geprüft.
- Speichergrenze nach EEPROM-/RAM-Budget festlegen, z.B. 32 Slots mit IDs 0..31.
  Ob EEPROM oder nur RAM wurde angefragt. EEPROM: Versionsnummer, Prüfsumme,
  Update nur auf ausdrückliches Speichern, robuste Schreibstrategie gegen
  Stromausfall und Erkennung inkompatibler Kalibrierstände.
- Gespeicherte Sollpose ist kein Beleg für eine gemessene aktuelle Position.
  Nach Neustart weiterhin Homing; reine Positionsdaten bleiben bei EEPROM erhalten.

Vorgeschlagene Bedienung (noch kein implementiertes neues Protokoll):

| Befehl | Bedeutung |
|---|---|
| HOME ALL | Alle Achsen im ausdrücklich konfigurierten Parallelmodus referenzieren |
| HOME ALL SEQUENTIAL | Alle Achsen nacheinander referenzieren |
| HOME J1 / HOME J2 / HOME J3 / HOME Z | Nur die genannte Achse referenzieren |
| SETJ J1 30 / SETZ 20 | Einzelne referenzierte Achse bewegen |
| MOVEJ 10 20 -30 40 3 | Bisheriges Format für mehrere Gelenke behalten |
| MOVEJ J1 10 J2 20 T 3 | Vorgeschlagene zusätzliche Teilachsen-Syntax |
| MOVEL 210 145 30 0 3 | Direkte geprüfte TCP-Gerade |
| SAVE 0 | Aktuelle referenzierte TCP-Pose in Slot 0 speichern |
| POSITION 0 / Position 1 | Gespeicherte Pose über die geprüfte Geradenplanung anfahren |
| LIST / DELETE 0 | Speicher anzeigen bzw. Position löschen |
| STATUS / STOP / RESETSTOP | Zustand lesen, stoppen und Fehler zurücksetzen |

## Phase 5 – Verifikation und Auslieferung

1. Hosttests für Mathematik, Einheiten, beide Zweige, Home-Referenzen und Grenzfälle.
2. Tests für inneres Loch, ganze Bahn, Winkelentrollung, stationäre Bewegungen,
   reine Z-/Orientierungsfahrt, zu kurze Dauer und Konfigurationswechsel.
3. Zustands-/Fehlerinjektion: paralleles und einzelnes Homing, defekte Schalter,
   Timeout, STOP in jeder Phase, Queue-Unterlauf und ISR-Fehler/DONE-Race.
4. Positionsspeichertests: ungültige IDs, leere Slots, CRC/Version, Neustart,
   abgebrochener Schreibvorgang und veränderte Kalibrierung.
5. Performance-Vergleich mit identischer Start-/Zielpose und dokumentierten
   Achsgrenzen; Planungszeit getrennt von Fahrzeit. Keine Behauptung, MOVEL müsse
   immer genauso schnell wie MOVEJ sein oder global zeitoptimal sein.
6. Mega-Build und numerische Tests mit AVR-ähnlicher 32-bit-Präzision; anschließend
   Logikanalysator-/Hardwaremessungen von ISR, STEP/DIR, Referenzwiederholung und TCP.
7. Nachweisbare Ergebnisse mit Analyse/Tests ins neue GitHub-Repository hochladen,
   Versionsstände und ZIPs dazu erzeugen. Kein Hardware-Upload ohne Auftrag.

## Vorgeschlagene Dateien

RobotConfig, Types, RobotState, Kinematics, CartesianPath, QuinticProfile,
TrajectoryPlanner, MotionExecutor/Movement, MotorController, Homing,
PositionStore, SerialCommands und main. Kleine Teilmodule dürfen zusammenbleiben,
wenn das Verständnis dadurch steigt; keine Klasse nur um einer Klasse willen.

## Abnahmekriterien vor „fertig“

- Neue Ergebnisse liegen nachvollziehbar im tatsächlich angelegten GitHub-Repository.
- POSITION 0 ruft ohne zusätzliche Koordinateneingabe die gespeicherte TCP-Pose ab.
- Jede Positionsfahrt ist eine zulässige Gerade innerhalb der vereinbarten Toleranz
  oder wird vor Impulsausgabe mit verständlichem Grund abgelehnt.
- Nach korrekt kalibriertem regulärem Homing ist kein manueller MOVEJ-Vorbefehl nötig.
- Einzel- und Parallel-Homing sind achsweise überwacht; Teilreferenzen werden nicht
  als Vollreferenz ausgegeben.
- U-002 ist mit protokollierten Plan-/Fahrzeiten und unveränderten Limits adressiert.
- Fehler-/STOP-Abschaltungen erzeugen keine falsche Erfolgsmeldung.
- Build, Simulation und reale Messungen werden getrennt berichtet.
